#!/bin/bash

# Configure the Docker image and the MkDocs directory
DOCKER_IMAGE="squidfunk/mkdocs-material"
MKDOCS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/iiwa-ros-doc" && pwd)"

# Define color codes for logging
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info()    { echo -e "${GREEN}[INFO]${NC}  $1"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC}  $1"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $1"; }


# Check for Docker and the MkDocs directory
if ! command -v docker &> /dev/null; then
    log_error "Docker не найден. Установи Docker и повтори."
    exit 1
fi

if [ ! -d "$MKDOCS_DIR" ]; then
    log_error "Папка mkdocs не найдена: $MKDOCS_DIR"
    exit 1
fi

if [ ! -f "$MKDOCS_DIR/mkdocs.yml" ]; then
    log_error "Файл mkdocs.yml не найден в $MKDOCS_DIR"
    exit 1
fi

# Determine the mode (build or serve)
MODE="${1:-build}"

case "$MODE" in
    build)
        log_info "Сборка документации (build)..."
        docker run --rm \
            -v "$MKDOCS_DIR:/docs" \
            "$DOCKER_IMAGE" build
        if [ $? -eq 0 ]; then
            log_info "Готово! Результат: $MKDOCS_DIR/site/"
        else
            log_error "Сборка завершилась с ошибкой."
            exit 1
        fi
        ;;

    serve)
        log_info "Запуск dev-сервера на http://localhost:8000 ..."
        log_warn "Остановка: Ctrl+C"
        docker run --rm -it \
            -p 8000:8000 \
            -v "$MKDOCS_DIR:/docs" \
            "$DOCKER_IMAGE" serve \
            --dev-addr=0.0.0.0:8000 \
            --watch /docs \
            --livereload
        ;;

    *)
        echo "Использование: $0 [build|serve]"
        echo "  build  — собрать статику в mkdocs/site/ (по умолчанию)"
        echo "  serve  — запустить live-preview на localhost:8000"
        exit 1
        ;;
esac