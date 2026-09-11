#!/usr/bin/env python3
"""Serve declared simulation truth through the grounding-evidence contract."""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
from pathlib import Path
from typing import Any, Dict

import rospkg
import rospy
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
        self.assessment_artifact_file = str(
            rospy.get_param(
                "~assessment_artifact_file",
                package_root + "/config/evidence/grounding_validity.simulation.json",
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
        self.assessment = self._load_assessment(self.assessment_artifact_file)
        runtime_dir = str(os.environ.get("ORACLE_RUNTIME_DIR", "") or "").strip()
        if not runtime_dir:
            raise RuntimeError("ORACLE_RUNTIME_DIR is required for evidence assessment")
        self.capture_root = Path(runtime_dir).expanduser().resolve() / "captures"
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
    def _load_assessment(path: str) -> Dict[str, Any]:
        with open(path, "rb") as stream:
            artifact_bytes = stream.read()
        document = json.loads(artifact_bytes.decode("utf-8"))
        if (
            not isinstance(document, dict)
            or int(document.get("schema_version", 0)) != 1
        ):
            raise RuntimeError("unsupported grounding assessment artifact")
        required = (
            "assessment_model_id",
            "decision_model_id",
            "calibration_id",
        )
        if any(not str(document.get(key, "") or "").strip() for key in required):
            raise RuntimeError("grounding assessment artifact is incomplete")
        if (
            str(document.get("scope", "") or "").strip().lower() != "simulation_only"
            or document.get("calibration_eligible_for_physical_use") is not False
        ):
            raise RuntimeError("grounding assessment artifact is not simulation-only")
        decisions = document.get("entity_decisions")
        if not isinstance(decisions, dict):
            raise RuntimeError("grounding assessment artifact requires entity rules")
        output = dict(document)
        output["artifact_digest"] = hashlib.sha256(artifact_bytes).hexdigest()
        return output

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

    def _validate_raw_capture(self, req: Any) -> str:
        requested_path = Path(str(req.raw_capture_path or "")).expanduser()
        if not requested_path.is_absolute() or requested_path.is_symlink():
            raise ValueError("assessment_raw_capture_path_invalid")
        path = requested_path.resolve(strict=True)
        try:
            path.relative_to(self.capture_root)
        except ValueError as exc:
            raise ValueError("assessment_raw_capture_outside_runtime") from exc
        if not path.is_file():
            raise ValueError("assessment_raw_capture_not_regular")
        archive_digest = hashlib.sha256()
        message_digest = hashlib.sha256()
        with path.open("rb") as stream:
            envelope_line = stream.readline(65536)
            if not envelope_line.endswith(b"\n"):
                raise ValueError("assessment_raw_capture_envelope_invalid")
            archive_digest.update(envelope_line)
            try:
                envelope = json.loads(envelope_line.decode("utf-8"))
            except (UnicodeDecodeError, ValueError) as exc:
                raise ValueError("assessment_raw_capture_envelope_invalid") from exc
            if not isinstance(envelope, dict):
                raise ValueError("assessment_raw_capture_envelope_invalid")
            embedded_digest = str(envelope.pop("content_digest", "") or "").lower()
            message_digest.update(
                json.dumps(
                    envelope,
                    sort_keys=True,
                    separators=(",", ":"),
                    allow_nan=False,
                ).encode("utf-8")
            )
            message_digest.update(b"\x00")
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                archive_digest.update(chunk)
                message_digest.update(chunk)
        archive_sha256 = archive_digest.hexdigest()
        content_digest = message_digest.hexdigest()
        declared_content = str(req.content_digest or "").lower()
        declared_archive = str(req.raw_capture_archive_sha256 or "").lower()
        expected_root = hashlib.sha256(
            "{}:{}".format(req.capture_id, declared_content).encode("utf-8")
        ).hexdigest()
        if (
            embedded_digest != declared_content
            or content_digest != declared_content
            or archive_sha256 != declared_archive
            or str(req.physical_root_id or "").lower() != expected_root
        ):
            raise ValueError("assessment_raw_capture_binding_mismatch")
        return archive_sha256

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
            "decision_calibration_id",
            "decision_calibration_artifact_digest",
            "assessment_model_id",
            "assessment_calibration_id",
            "assessment_calibration_artifact_digest",
            "expected_world_id",
            "expected_source_incarnation",
            "expected_world_event_digest",
            "expected_entity_digest",
            "physical_root_id",
            "capture_id",
            "content_digest",
            "raw_capture_path",
            "raw_capture_archive_sha256",
            "capture_observation_label",
            "sensor_id",
            "evidence_source_incarnation",
            "capture_model_id",
            "capture_calibration_id",
            "capture_calibration_artifact_digest",
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
            req.decision_calibration_artifact_digest,
            req.assessment_calibration_artifact_digest,
            req.expected_world_event_digest,
            req.expected_entity_digest,
            req.content_digest,
            req.raw_capture_archive_sha256,
            req.capture_calibration_artifact_digest,
        ):
            if not self._sha256(value):
                return self._failure("assessment_digest_invalid")
        try:
            assessed_archive_sha256 = self._validate_raw_capture(req)
        except (OSError, TypeError, ValueError) as exc:
            return self._failure(str(exc))
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
        decision_model_id = str(req.decision_model_id).strip()
        assessment_model_id = str(req.assessment_model_id).strip()
        calibration_id = str(req.assessment_calibration_id).strip()
        calibration_digest = str(req.assessment_calibration_artifact_digest).strip()
        if (
            decision_model_id != str(self.assessment.get("decision_model_id", "") or "")
            or assessment_model_id
            != str(self.assessment.get("assessment_model_id", "") or "")
            or calibration_id != str(self.assessment.get("calibration_id", "") or "")
            or calibration_digest
            != str(self.assessment.get("artifact_digest", "") or "")
        ):
            return self._failure("assessment_contract_mismatch")
        decisions = self.assessment["entity_decisions"]
        entry = decisions.get(target_id)
        if entry is None:
            entry = {
                "decision": self.assessment.get("default_decision", "defer"),
                "confidence": self.assessment.get("default_confidence", 0.0),
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
                "decision_model_id": decision_model_id,
                "decision_calibration_id": str(req.decision_calibration_id),
                "decision_calibration_artifact_digest": str(
                    req.decision_calibration_artifact_digest
                ),
                "assessment_model_id": assessment_model_id,
                "assessment_calibration_id": calibration_id,
                "assessment_calibration_artifact_digest": calibration_digest,
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
                    "raw_capture_archive_sha256": assessed_archive_sha256,
                    "capture_observation_label": str(req.capture_observation_label),
                    "sensor_id": str(req.sensor_id),
                    "source_incarnation": str(req.evidence_source_incarnation),
                },
                "capture_model_id": str(req.capture_model_id),
                "capture_calibration_id": str(req.capture_calibration_id),
                "capture_calibration_artifact_digest": str(
                    req.capture_calibration_artifact_digest
                ),
                "decision": decision,
                "confidence": confidence,
                "assessment_rule_digest": _digest(self.assessment),
            }
        )
        return AssessGroundingEvidenceResponse(
            accepted=True,
            reason="assessment_complete",
            decision=decision,
            confidence=confidence,
            evidence_digest=evidence_digest,
            assessed_capture_archive_sha256=assessed_archive_sha256,
            assessment_model_id=assessment_model_id,
            assessment_calibration_id=calibration_id,
            assessment_calibration_artifact_digest=calibration_digest,
        )


def main() -> None:
    rospy.init_node("grounding_evidence_assessor")
    SimulatedGroundingEvidenceAssessor()
    rospy.spin()


if __name__ == "__main__":
    main()
