from dataclasses import dataclass, field


@dataclass
class PreCheckItemResult:
    item_name: str
    passed: bool
    error_message: str | None = None


@dataclass
class PreCheckResult:
    passed: bool
    items: list[PreCheckItemResult] = field(default_factory=list)

    @property
    def failed_items(self) -> list[PreCheckItemResult]:
        return [i for i in self.items if not i.passed]
