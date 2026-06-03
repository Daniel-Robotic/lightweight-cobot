from fastapi import APIRouter
from pydantic import BaseModel, Field

from .config_loader import load_named_positions

router = APIRouter(tags=["robot"])


class NamedPositionResponse(BaseModel):
    name: str = Field(
        description=(
            "Идентификатор позиции. Передайте это значение в поле `name` запроса "
            "`POST /robot/move/named`, чтобы переместить робота в эту позу."
        )
    )
    group: str = Field(
        description="Группа планирования MoveIt, к которой относится позиция (обычно `iiwa_arm`)."
    )
    joints: dict[str, float] = Field(
        description=(
            "Целевые углы суставов в радианах. Ключи: joint1..joint7. "
            "Используйте эти значения для оценки конфигурации перед движением "
            "или как основу для `POST /robot/move/joints`."
        )
    )
    description: str = Field(
        description="Человекочитаемое описание назначения позиции."
    )


_srdf_path: str | None = None
_meta_path: str | None = None


def init(srdf_path: str | None = None, meta_path: str | None = None) -> None:
    global _srdf_path, _meta_path
    _srdf_path = srdf_path
    _meta_path = meta_path


@router.get(
    "/robot/positions",
    response_model=list[NamedPositionResponse],
    summary="Список заготовленных именованных позиций робота",
    description=(
        "Возвращает все именованные позиции (`group_state`) из SRDF-файла конфигурации робота. "
        "\n\n"
        "**Типичный рабочий процесс для агента:**\n"
        "1. Вызовите этот эндпоинт, чтобы узнать доступные позиции и их суставные значения.\n"
        "2. Выберите подходящую позицию по полю `description` и значениям `joints`.\n"
        "3. Передайте поле `name` выбранной позиции в `POST /robot/move/named`, "
        "чтобы переместить робота туда.\n"
        "\n"
        "Позиции определены статически в SRDF и гарантированно безопасны с точки зрения "
        "столкновений и кинематики."
    ),
)
def get_named_positions() -> list[NamedPositionResponse]:
    positions = load_named_positions(srdf_path=_srdf_path, meta_path=_meta_path)
    return [
        NamedPositionResponse(
            name=p.name,
            group=p.group,
            joints=p.joints,
            description=p.description,
        )
        for p in positions
    ]
