#!/usr/bin/env python3

"""
@package correct_permissions
Verifica e corrige a permissão dos arquivos python do projeto agrobot.
"""

from os import system
from pathlib import Path

# Variáveis de diretório.
current_dir: str = str(Path(__file__).parent.absolute()) + "/"
project_src_dir: str = current_dir + '../../../src/agrobot/src/'
folders_to_correct: tuple = (
    project_src_dir,
    project_src_dir + 'robot_nodes/',
    project_src_dir + 'robot_services/',
    project_src_dir + 'robot_utils/'
)

def correct_permissions(folders: tuple) -> None:
    """Corrige a permissão dos diretórios passados como parâmetro."""
    for dir in folders:
        system('cd {0} && chmod +x *.py'.format(dir))

if __name__ == '__main__':
    try:
        correct_permissions(folders_to_correct)
    except Exception as e:
        print('Could not run correct_permissions.py properly. {0}.'.format(e))