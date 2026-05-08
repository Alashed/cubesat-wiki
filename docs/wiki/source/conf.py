# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'CubeSat 1U Educational Kit'
copyright = '2026, AlashNabor'
author = 'AlashNabor'

version = '0.1.0'
release = '0.1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx_copybutton',
]

templates_path = ['_templates']
exclude_patterns = []

# Подсветка кода в .ino-скетчах по умолчанию как C++.
highlight_language = 'cpp'


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

language = 'ru'

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

html_theme_options = {
    'navigation_depth': 3,
    'collapse_navigation': False,
    'sticky_navigation': True,
}

# sphinx-copybutton: не копировать строки с приглашением (>>>, $) и вывод.
copybutton_prompt_text = r'>>> |\.\.\. |\$ |# '
copybutton_prompt_is_regexp = True
