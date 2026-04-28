# CubeSat 1U Wiki (Sphinx)

Документация проекта находится в `docs/wiki/source`.

## Быстрый старт

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r docs/wiki/requirements.txt
.venv/bin/sphinx-build -b html docs/wiki/source docs/wiki/build/html
```

Готовые HTML-страницы: `docs/wiki/build/html/index.html`.
