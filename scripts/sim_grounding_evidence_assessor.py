#!/usr/bin/env python3
"""Serve declared simulation truth through the grounding-evidence contract."""

from __future__ import annotations

import hashlib
import json
import math
import re
from typing import Any, Dict

import rospkg
import rospy
import yaml

from oracle.srv import AssessGroundingEvidence, AssessGroundingEvidenceResponse


def _digest(value: Any) -> str:
    encoded = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


class SimulatedGroundingEvidenceAssessor:
    """Resolve one target-validity question from declared simulator truth."""

    _DECISIONS = {"support", "reject", "defer"}

    def __init__(self) -> None:
        runtime_mode = (
            str(rospy.get_param("/grande/runtime/mode", "unknown") or "unknown")
            .strip()
            .lower()
        )
        if runtime_mode != "sim":
            raise RuntimeError("simulated grounding assessor requires sim mode")
        package_root = rospkg.RosPack().get_path("heron_simulator")
        self.models_file = str(
            rospy.get_param(
                "~models_file",
                package_root + "/config/grounding_evidence_models.yaml",
            )
            or ""
        ).strip()
        self.allowed_caller = str(
            rospy.get_param("~allowed_caller", "/manager") or "/manager"
        ).strip()
        self.max_evidence_age_sec = float(
            rospy.get_param("~max_evidence_age_sec", 30.0)
        )
        self.future_skew_sec = float(rospy.get_param("~future_skew_sec", 0.25))
        if (
            not math.isfinite(self.max_evidence_age_sec)
            or self.max_evidence_age_sec <= 0.0
            or not math.isfinite(self.future_skew_sec)
            or self.future_skew_sec < 0.0
        ):
            raise RuntimeError("grounding evidence age limits are invalid")
        self.models = self._load_models(self.models_file)
        service_name = str(
            rospy.get_param("~service_name", "/oracle/grounding_evidence/assess")
            or "/oracle/grounding_evidence/assess"
        ).strip()
        self.service = rospy.Service(
            service_name,
            AssessGroundingEvidence,
            self._handle,
        )

    @staticmethod
    def _load_models(path: str) -> Dict[str, Dict[str, Any]]:
        with open(path, "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        if (
            not isinstance(document, dict)
            or int(document.get("schema_version", 0)) != 1
        ):
            raise RuntimeError("unsupported grounding evidence catalog schema")
        models = document.get("models")
        if not isinstance(models, dict):
            raise RuntimeError("grounding evidence catalog requires models")
        return {str(key): dict(value or {}) for key, value in models.items()}

    @staticmethod
    def _caller(req: Any) -> str:
        header = getattr(req, "_connection_header", {}) or {}
        return str(header.get("callerid", "") or "").strip()

    @staticmethod
    def _failure(reason: str) -> AssessGroundingEvidenceResponse:
        return AssessGroundingEvidenceResponse(
            accepted=False,
            reason=str(reason),
            decision="defer",
            confidence=0.0,
            evidence_digest="",
        )

    @staticmethod
    def _sha256(value: Any) -> bool:
        return bool(re.fullmatch(r"[0-9a-f]{64}", str(value or "").lower()))

    def _handle(self, req: Any) -> AssessGroundingEvidenceResponse:
        if self.allowed_caller and self._caller(req) != self.allowed_caller:
            return self._failure("caller_not_authorized")
        required = (
            "request_id",
            "authorization_digest",
            "authorization_nonce",
            "dependency_key",
            "target_entity_id",
            "decision_model_id",
            "expected_world_id",
            "expected_source_incarnation",
            "expected_world_event_digest",
            "expected_entity_digest",
            "physical_root_id",
            "capture_id",
            "content_digest",
            "sensor_id",
            "evidence_source_incarnation",
            "observation_model_id",
            "calibration_id",
            "calibration_artifact_digest",
        )
        if any(not str(getattr(req, key, "") or "").strip() for key in required):
            return self._failure("assessment_contract_incomplete")
        target_id = str(req.target_entity_id).strip()
        if str(req.dependency_key).strip() != "entity:" + target_id:
            return self._failure("assessment_dependency_scope_unsupported")
        if int(req.expected_world_revision) <= 0:
            return self._failure("assessment_world_revision_invalid")
        for value in (
            req.authorization_digest,
            req.expected_world_event_digest,
            req.expected_entity_digest,
            req.content_digest,
            req.calibration_artifact_digest,
        ):
            if not self._sha256(value):
                return self._failure("assessment_digest_invalid")
        stamp = float(req.header.stamp.to_sec())
        now = float(rospy.Time.now().to_sec())
        if (
            not math.isfinite(stamp)
            or stamp <= 0.0
            or not math.isfinite(now)
            or now <= 0.0
            or now - stamp > self.max_evidence_age_sec
            or stamp - now > self.future_skew_sec
        ):
            return self._failure("assessment_evidence_stale")
        model_id = str(req.decision_model_id).strip()
        model = dict(self.models.get(model_id, {}) or {})
        modes = {
            str(item or "").strip().lower()
            for item in list(model.get("runtime_modes", []) or [])
        }
        if not model or not bool(model.get("enabled", False)):
            return self._failure(
                str(model.get("unavailable_reason", "") or "assessment_model_disabled")
            )
        if modes != {"sim"}:
            return self._failure("assessment_runtime_mode_mismatch")
        decisions = model.get("entity_decisions", {}) or {}
        if not isinstance(decisions, dict):
            return self._failure("assessment_model_invalid")
        entry = decisions.get(target_id)
        if entry is None:
            entry = {
                "decision": model.get("default_decision", "defer"),
                "confidence": model.get("default_confidence", 0.0),
            }
        if not isinstance(entry, dict):
            return self._failure("assessment_target_rule_invalid")
        decision = str(entry.get("decision", "defer") or "defer").strip().lower()
        try:
            confidence = float(entry.get("confidence", 0.0))
        except (TypeError, ValueError):
            return self._failure("assessment_confidence_invalid")
        if (
            decision not in self._DECISIONS
            or not math.isfinite(confidence)
            or not 0.0 <= confidence <= 1.0
        ):
            return self._failure("assessment_target_rule_invalid")
        evidence_digest = _digest(
            {
                "schema_version": 1,
                "request_id": str(req.request_id),
                "authorization_digest": str(req.authorization_digest),
                "authorization_nonce": str(req.authorization_nonce),
                "dependency_key": str(req.dependency_key),
                "target_entity_id": target_id,
                "decision_model_id": model_id,
                "world": {
                    "id": str(req.expected_world_id),
                    "incarnation": str(req.expected_source_incarnation),
                    "revision": int(req.expected_world_revision),
                    "event_digest": str(req.expected_world_event_digest),
                    "entity_digest": str(req.expected_entity_digest),
                },
                "root": {
                    "physical_root_id": str(req.physical_root_id),
                    "capture_id": str(req.capture_id),
                    "content_digest": str(req.content_digest),
                    "sensor_id": str(req.sensor_id),
                    "source_incarnation": str(req.evidence_source_incarnation),
                },
                "observation_model_id": str(req.observation_model_id),
                "calibration_id": str(req.calibration_id),
                "calibration_artifact_digest": str(req.calibration_artifact_digest),
                "decision": decision,
                "confidence": confidence,
                "model_rule_digest": _digest(model),
            }
        )
        return AssessGroundingEvidenceResponse(
            accepted=True,
            reason="assessment_complete",
            decision=decision,
            confidence=confidence,
            evidence_digest=evidence_digest,
        )


def main() -> None:
    rospy.init_node("grounding_evidence_assessor")
    SimulatedGroundingEvidenceAssessor()
    rospy.spin()


if __name__ == "__main__":
    main()
