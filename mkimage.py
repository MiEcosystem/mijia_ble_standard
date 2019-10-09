#!/usr/bin/env python3
import sys, os
import argparse

sys.path.append(os.path.join(os.path.dirname(__file__), '../../../../utilities/', 'python_scripts'))
from api.script_base import ProductId
from suota.v11.mkimage import mkimage

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('build_configuration', type=str, help='Build configuration name')
    parser.add_argument('--prod_id', '-p', required=True, type=str, help='Device product id',
                        choices=[p.value for p in ProductId])
    args = parser.parse_args()

    mkimage('{}/pxp_reporter.bin'.format(args.build_configuration), prod_id=args.prod_id)
