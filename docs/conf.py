# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os 
import subprocess 
import sys 

sys.path.insert(0, os.path.abspath('..'))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'surveillance_robot'
copyright = '2022, Ettore Sani'
author = 'Ettore Sani'
release = '0.0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html

extensions = [ 'sphinx.ext.autodoc', 
			   'sphinx.ext.doctest', 
			   'sphinx.ext.intersphinx', 
			   'sphinx.ext.todo', 
			   'sphinx.ext.coverage', 
			   'sphinx.ext.mathjax', 
			   'sphinx.ext.ifconfig', 
			   'sphinx.ext.viewcode', 
			   'sphinx.ext.githubpages', 
			   'sphinx.ext.napoleon', 
			   'sphinx.ext.inheritance_diagram',
				]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

autodoc_default_options = {"members": True, "undoc-members": True, "private-members": True}

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

master_doc = 'index'
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']