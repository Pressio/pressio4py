#!/bin/bash

python3 ./unsteady_default_galerkin_advdiff1d_pod/main.py
python3 ./unsteady_default_lspg_advdiff1d_kpca/main.py
python3 ./unsteady_default_lspg_advdiff1d_pod/main.py
python3 ./unsteady_masked_galerkin_advdiff1d_pod/main.py
python3 ./unsteady_masked_galerkin_vs_lspg_advdiff1d_pod/main.py
