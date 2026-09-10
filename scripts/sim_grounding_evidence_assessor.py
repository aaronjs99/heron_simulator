#!/usr/bin/env python3
"""Serve declared simulation truth through the grounding-evidence contract."""

from __future__ import annotations

import hashlib
import json
import math
import re
from typing import Any, Dict

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import yaml

from oracle.srv import AssessGroundingEvidence


def _digest(value: Any) -> str:
    encoded = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


class SimulatedGroundingEvidenceAssessor(Node):
    """Resolve one target-validity question from declared simulator truth."""

    _DECISIONS = {"support", "reject", "defer"}

    def __init__(self) -> None:
        super().__init__("grounding_evidence_assessor")

        runtime_mode = (
            str(self.declare_parameter("sim_mode", "sim").value or "unknown")
            .strip()
            .lower()
        )
        if runtime_mode != "sim":
            raise RuntimeError("simulated grounding assessor requires sim mode")
        package_root = get_package_share_directory("heron_simulator")
        self.models_file = str(
            self.declare_parameter(
                "models_file",
                package_root + "/config/grounding_evidence_models.yaml",
            ).value
            or ""
        ).strip()
        self.allowed_caller = str(
            self.declare_parameter("allowed_caller", "/manager").value or "/manager"
        ).strip()
        self.max_evidence_age_sec = float(
            self.declare_parameter("max_evidence_age_sec", 30.0).value
        )
        self.future_skew_sec = float(self.declare_parameter("future_skew_sec", 0.25).value)
        if (
            not math.isfinite(self.max_evidence_age_sec)
            or self.max_evidence_age_sec <= 0.0
            or not math.isfinite(self.future_skew_sec)
            or self.future_skew_sec < 0.0
        ):
            raise RuntimeError("grounding evidence age limits are invalid")
        self.models = self._load_models(self.models_file)
        service_name = str(
            self.declare_parameter(
                "service_name", "/oracle/grounding_evidence/assess"
            ).value
            or "/oracle/grounding_evidence/assess"
        ).strip()
        self.service = self.create_service(
            AssessGroundingEvidence,
            service_name,
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

    # TODO: figure out correct ROS2 conversion for _connection_header
    # @staticmethod
    # def _caller(req: Any) -> str:
    #     header = getattr(req, "_connection_header", {}) or {}
    #     return str(header.get("callerid", "") or "").strip()

    @staticmethod
    def _sha256(value: Any) -> bool:
        return bool(re.fullmatch(r"[0-9a-f]{64}", str(value or "").lower()))

    def _fail(self, response, reason: str):
        response.accepted = False
        response.reason = str(reason)
        response.decision = "defer"
        response.confidence = 0.0
        response.evidence_digest = ""
        return response

    def _handle(self, req: Any, response: Any) -> Any:
        # NOTE: caller-identity authorization check removed here, will add once converted
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
            return self._fail(response, "assessment_contract_incomplete")
        target_id = str(req.target_entity_id).strip()
        if str(req.dependency_key).strip() != "entity:" + target_id:
            return self._fail(response, "assessment_dependency_scope_unsupported")
        if int(req.expected_world_revision) <= 0:
            return self._fail(response, "assessment_world_revision_invalid")
        for value in (
            req.authorization_digest,
            req.expected_world_event_digest,
            req.expected_entity_digest,
            req.content_digest,
            req.calibration_artifact_digest,
        ):
            if not self._sha256(value):
                return self._fail(response, "assessment_digest_invalid")
        stamp = Time.from_msg(req.header.stamp).nanoseconds / 1e9
        now = self.get_clock().now().nanoseconds / 1e9
        if (
            not math.isfinite(stamp)
            or stamp <= 0.0
            or not math.isfinite(now)
            or now <= 0.0
            or now - stamp > self.max_evidence_age_sec
            or stamp - now > self.future_skew_sec
        ):
            return self._fail(response, "assessment_evidence_stale")
        model_id = str(req.decision_model_id).strip()
        model = dict(self.models.get(model_id, {}) or {})
        modes = {
            str(item or "").strip().lower()
            for item in list(model.get("runtime_modes", []) or [])
        }
        if not model or not bool(model.get("enabled", False)):
            return self._fail(
                response,
                str(model.get("unavailable_reason", "") or "assessment_model_disabled"),
            )
        if modes != {"sim"}:
            return self._fail(response, "assessment_runtime_mode_mismatch")
        decisions = model.get("entity_decisions", {}) or {}
        if not isinstance(decisions, dict):
            return self._fail(response, "assessment_model_invalid")
        entry = decisions.get(target_id)
        if entry is None:
            entry = {
                "decision": model.get("default_decision", "defer"),
                "confidence": model.get("default_confidence", 0.0),
            }
        if not isinstance(entry, dict):
            return self._fail(response, "assessment_target_rule_invalid")
        decision = str(entry.get("decision", "defer") or "defer").strip().lower()
        try:
            confidence = float(entry.get("confidence", 0.0))
        except (TypeError, ValueError):
            return self._fail(response, "assessment_confidence_invalid")
        if (
            decision not in self._DECISIONS
            or not math.isfinite(confidence)
            or not 0.0 <= confidence <= 1.0
        ):
            return self._fail(response, "assessment_target_rule_invalid")
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
        response.accepted = True
        response.reason = "assessment_complete"
        response.decision = decision
        response.confidence = confidence
        response.evidence_digest = evidence_digest
        return response


def main() -> None:
    rclpy.init()
    node = SimulatedGroundingEvidenceAssessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()