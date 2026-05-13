#!/usr/bin/env python3
"""Runtime topic contract guard for the Heron simulation stack.

This node validates a module-based topic contract loaded under /topic_contract.
The contract is intentionally data-driven so new stacks (DLIO, Mariner,
ig_handle sensors, Oracle, etc.) can add endpoints without rewriting the guard.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Tuple

import rosgraph
import rospy


def normalize_topic(topic: str) -> str:
    if not topic:
        return topic
    return topic if topic.startswith("/") else f"/{topic}"


def normalize_topic_map(
    entries: Iterable[Tuple[str, List[str]]]
) -> Dict[str, List[str]]:
    return {normalize_topic(topic): list(nodes) for topic, nodes in entries}


@dataclass(frozen=True)
class CountRule:
    minimum: int = 0
    maximum: Optional[int] = None


@dataclass(frozen=True)
class EndpointRule:
    module: str
    name: str
    topic: str
    expected_type: Optional[str] = None
    publishers: Optional[CountRule] = None
    subscribers: Optional[CountRule] = None


@dataclass(frozen=True)
class ModuleRule:
    name: str
    enabled: bool
    endpoints: List[EndpointRule]


@dataclass(frozen=True)
class ForbiddenTopicRule:
    name: str
    topic: str
    reason: str


@dataclass(frozen=True)
class ServiceRule:
    name: str
    service: str
    expected_type: Optional[str] = None
    servers: Optional[CountRule] = None
    clients: Optional[CountRule] = None
    optional: bool = False


@dataclass(frozen=True)
class TopicContract:
    topics: Dict[str, str]
    type_aliases: Dict[str, str]
    services: Dict[str, ServiceRule]
    modules: List[ModuleRule]
    forbidden_topics: List[ForbiddenTopicRule]
    chains: Dict[str, List[str]] = field(default_factory=dict)


def _coerce_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _parse_count_rule(config: Optional[Dict[str, Any]]) -> Optional[CountRule]:
    if not config:
        return None
    minimum = int(config.get("min", 0))
    maximum = config.get("max")
    if maximum is not None:
        maximum = int(maximum)
    return CountRule(minimum=minimum, maximum=maximum)


def _resolve_topic(
    topics: Dict[str, str],
    endpoint_config: Dict[str, Any],
    module_name: str,
    endpoint_name: str,
) -> str:
    topic = endpoint_config.get("topic")
    topic_key = endpoint_config.get("topic_key")
    if topic_key:
        if topic_key not in topics:
            raise ValueError(
                f"module {module_name}.{endpoint_name} references unknown topic_key {topic_key}"
            )
        topic = topics[topic_key]
    if not topic:
        raise ValueError(
            f"module {module_name}.{endpoint_name} is missing topic/topic_key"
        )
    return normalize_topic(topic)


def _resolve_type(
    type_aliases: Dict[str, str],
    endpoint_config: Dict[str, Any],
    module_name: str,
    endpoint_name: str,
) -> Optional[str]:
    expected_type = endpoint_config.get("type")
    type_key = endpoint_config.get("type_key")
    if type_key:
        if type_key not in type_aliases:
            raise ValueError(
                f"module {module_name}.{endpoint_name} references unknown type_key {type_key}"
            )
        expected_type = type_aliases[type_key]
    return expected_type


def _resolve_forbidden_topic(name: str, rule_config: Dict[str, Any]) -> str:
    if "topic" in rule_config:
        return normalize_topic(rule_config.get("topic", ""))

    topic_parts = rule_config.get("topic_parts")
    if topic_parts is None:
        return ""
    if not isinstance(topic_parts, (list, tuple)):
        raise ValueError(f"forbidden topic rule {name} topic_parts must be a list")
    return normalize_topic(
        "/".join(str(part).strip("/") for part in topic_parts if part)
    )


def _resolve_service(name: str, rule_config: Dict[str, Any]) -> str:
    service = str(rule_config.get("service", "") or "")
    if not service:
        raise ValueError(f"service rule {name} is missing service")
    return normalize_topic(service)


def load_topic_contract(config: Dict[str, Any]) -> TopicContract:
    topics = {
        name: normalize_topic(topic)
        for name, topic in dict(config.get("topics", {})).items()
        if topic
    }
    type_aliases = dict(config.get("types", {}))

    services: Dict[str, ServiceRule] = {}
    for service_name, service_config in dict(config.get("services", {})).items():
        if not isinstance(service_config, dict):
            raise ValueError(f"service rule {service_name} must be a mapping")
        services[service_name] = ServiceRule(
            name=service_name,
            service=_resolve_service(service_name, service_config),
            expected_type=service_config.get("type"),
            servers=_parse_count_rule(service_config.get("servers")),
            clients=_parse_count_rule(service_config.get("clients")),
            optional=_coerce_bool(service_config.get("optional", False)),
        )

    modules: List[ModuleRule] = []
    for module_name, module_config in dict(config.get("modules", {})).items():
        endpoints: List[EndpointRule] = []
        for endpoint_name, endpoint_config in dict(
            module_config.get("endpoints", {})
        ).items():
            topic = _resolve_topic(topics, endpoint_config, module_name, endpoint_name)
            expected_type = _resolve_type(
                type_aliases, endpoint_config, module_name, endpoint_name
            )
            endpoints.append(
                EndpointRule(
                    module=module_name,
                    name=endpoint_name,
                    topic=topic,
                    expected_type=expected_type,
                    publishers=_parse_count_rule(endpoint_config.get("publishers")),
                    subscribers=_parse_count_rule(endpoint_config.get("subscribers")),
                )
            )
        modules.append(
            ModuleRule(
                name=module_name,
                enabled=_coerce_bool(module_config.get("enabled", True)),
                endpoints=endpoints,
            )
        )

    forbidden_topics: List[ForbiddenTopicRule] = []
    for name, rule_config in dict(config.get("forbidden_topics", {})).items():
        if isinstance(rule_config, str):
            topic = normalize_topic(rule_config)
            reason = "topic is forbidden by the contract"
        else:
            topic = _resolve_forbidden_topic(name, rule_config)
            reason = str(
                rule_config.get("reason", "topic is forbidden by the contract")
            )
        if not topic:
            raise ValueError(f"forbidden topic rule {name} is missing topic")
        forbidden_topics.append(
            ForbiddenTopicRule(name=name, topic=topic, reason=reason)
        )

    chains = {
        chain_name: list(chain_steps)
        for chain_name, chain_steps in dict(config.get("chains", {})).items()
    }
    return TopicContract(
        topics=topics,
        type_aliases=type_aliases,
        services=services,
        modules=modules,
        forbidden_topics=forbidden_topics,
        chains=chains,
    )


def validate_topic_contract(
    publishers: Dict[str, List[str]],
    subscribers: Dict[str, List[str]],
    topic_types: Dict[str, str],
    contract: TopicContract,
    services: Optional[Dict[str, List[str]]] = None,
    service_types: Optional[Dict[str, str]] = None,
) -> List[str]:
    errors: List[str] = []
    services = services or {}
    service_types = service_types or {}

    def pub_count(topic: str) -> int:
        return len(publishers.get(topic, []))

    def sub_count(topic: str) -> int:
        return len(subscribers.get(topic, []))

    def service_server_count(service: str) -> int:
        return len(services.get(service, []))

    def validate_count(topic: str, actual: int, rule: CountRule, noun: str) -> None:
        if actual < rule.minimum:
            errors.append(
                f"{topic} has {actual} {noun}, expected at least {rule.minimum}"
            )
        if rule.maximum is not None and actual > rule.maximum:
            errors.append(
                f"{topic} has {actual} {noun}, expected at most {rule.maximum}"
            )

    for module in contract.modules:
        if not module.enabled:
            continue
        for endpoint in module.endpoints:
            if endpoint.publishers is not None:
                validate_count(
                    endpoint.topic,
                    pub_count(endpoint.topic),
                    endpoint.publishers,
                    "publishers",
                )
            if endpoint.subscribers is not None:
                validate_count(
                    endpoint.topic,
                    sub_count(endpoint.topic),
                    endpoint.subscribers,
                    "subscribers",
                )
            if endpoint.expected_type:
                actual_type = topic_types.get(endpoint.topic)
                if actual_type and actual_type != endpoint.expected_type:
                    errors.append(
                        f"{endpoint.topic} has type {actual_type}, expected {endpoint.expected_type}"
                    )

    for rule in contract.forbidden_topics:
        if pub_count(rule.topic) > 0 or sub_count(rule.topic) > 0:
            errors.append(f"forbidden topic {rule.topic} is active: {rule.reason}")

    for service_rule in contract.services.values():
        if service_rule.optional and service_server_count(service_rule.service) == 0:
            continue
        if service_rule.servers is not None:
            validate_count(
                service_rule.service,
                service_server_count(service_rule.service),
                service_rule.servers,
                "service servers",
            )
        if service_rule.expected_type:
            actual_type = service_types.get(service_rule.service)
            if actual_type and actual_type != service_rule.expected_type:
                errors.append(
                    f"{service_rule.service} has service type {actual_type}, expected {service_rule.expected_type}"
                )

    return errors


class TopicContractGuard:
    def __init__(self) -> None:
        self.master = rosgraph.Master(rospy.get_name())
        self.poll_period = float(rospy.get_param("~poll_period_sec", 2.0))

        contract_ns = rospy.get_param("~contract_ns", "/topic_contract")
        contract_config = rospy.get_param(contract_ns, {})
        self.contract = load_topic_contract(contract_config)

        self._last_error_signature = None
        self._timer = rospy.Timer(rospy.Duration(self.poll_period), self._tick)

        enabled_modules = [
            module.name for module in self.contract.modules if module.enabled
        ]
        rospy.loginfo(
            "topic_contract_guard: monitoring modules=%s",
            ", ".join(enabled_modules) if enabled_modules else "(none)",
        )

    def _snapshot(self):
        publishers, subscribers, services = self.master.getSystemState()
        topic_types = {
            normalize_topic(topic): topic_type
            for topic, topic_type in self.master.getTopicTypes()
        }
        return (
            normalize_topic_map(publishers),
            normalize_topic_map(subscribers),
            topic_types,
            normalize_topic_map(services),
            {},
        )

    def _tick(self, _event) -> None:
        try:
            publishers, subscribers, topic_types, services, service_types = (
                self._snapshot()
            )
            errors = validate_topic_contract(
                publishers=publishers,
                subscribers=subscribers,
                topic_types=topic_types,
                services=services,
                service_types=service_types,
                contract=self.contract,
            )
        except Exception as exc:
            rospy.logerr("topic_contract_guard: failed to inspect graph: %s", exc)
            rospy.signal_shutdown(f"topic contract inspection failed: {exc}")
            return

        if not errors:
            self._last_error_signature = None
            return

        signature = tuple(errors)
        if signature != self._last_error_signature:
            rospy.logerr("topic_contract_guard: contract violated")
            for err in errors:
                rospy.logerr("topic_contract_guard: %s", err)
            self._last_error_signature = signature
        rospy.signal_shutdown("topic contract violated")


def main() -> None:
    rospy.init_node("topic_contract_guard")
    TopicContractGuard()
    rospy.spin()


if __name__ == "__main__":
    main()
