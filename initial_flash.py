#!/usr/bin/env python3
import sys, os
import argparse

sys.path.append(os.path.join(os.path.dirname(__file__), '../../../../utilities/', 'python_scripts'))
from api.script_base import ProductId


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('build_configuration', type=str, help='Build configuration name')
    parser.add_argument('--prod_id', '-p', required=True, type=str, help='Device product id',
                        choices=[p.value for p in ProductId])
    args = parser.parse_args()

    os.system('python ../../../../utilities/python_scripts/suota/v11/initial_flash.py '
              '{}/pxp_reporter.bin --prod_id {}'.format(args.build_configuration, args.prod_id))
