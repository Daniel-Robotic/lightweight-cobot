# Источники навыков проекта

Каталог навыков общий для Codex, OpenCode и BB. Навыки установлены 2026-11-18 из ветки `main`; идентификаторы commit фиксируют исходное состояние. Для обновления сначала изучите `SKILL.md` и сопутствующие файлы источника, затем обновите commit и список ниже.

| Репозиторий | Commit | Установленные навыки |
|---|---|---|
| [Fission-AI/OpenSpec](https://github.com/Fission-AI/OpenSpec) | Установленный CLI `openspec` 1.16.0; workflow skills уже находились в проекте | `openspec-propose`, `openspec-explore`, `openspec-apply-change`, `openspec-update-change`, `openspec-sync-specs`, `openspec-archive-change` |
| [JayRHa/agent-skills](https://github.com/JayRHa/agent-skills) | `f9a456b` | `github-actions-builder` |
| [tomzx/agents](https://github.com/tomzx/agents) | `31f0fc7` | `setup-docs-site` |
| [docker/skills](https://github.com/docker/skills) | `a65ae5b` | `docker-agent-config`, `docker-agent-deploy`, `docker-agent-run`, `docker-build-strategies`, `docker-compose-patterns`, `docker-destructive-guardrails`, `docker-docs`, `docker-project-foundations`, `docker-sandboxes-kits`, `docker-sandboxes-lifecycle`, `docker-sandboxes-network-credentials` |
| [petrkindlmann/qa-skills](https://github.com/petrkindlmann/qa-skills) | `5087a34` | `playwright-automation`, `agentic-browser-testing` |
| [microsoft/cat-agent-skills](https://github.com/microsoft/cat-agent-skills) | `189409b` | `browser-uat-analyst` |
| [pkuppens/skills](https://github.com/pkuppens/skills) | `6f3daac` | `readme-authoring` |

Все перечисленные навыки находятся в отдельных каталогах `.agents/skills/<skill-id>/` и имеют `SKILL.md`. Установка делается в репозитории, поэтому навыки доступны агентам, которые обнаруживают стандартный путь `.agents/skills/`. Секреты и настройки моделей в этот каталог не входят.
