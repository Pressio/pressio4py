
#-----------------------
# galerkin
#-----------------------
set(testdir1src  ${CMAKE_CURRENT_SOURCE_DIR}/burgers1d_galerkin)
set(testdir1dest ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/burgers1d_galerkin)
file(MAKE_DIRECTORY ${testdir1dest})
# needs the Burgers1d app, the basis file and test main driver
configure_file(${testdir1src}/test_burgers1d_galerkin.py
  ${testdir1dest} COPYONLY)
configure_file(${PRESSIO4PY_APPS_DIR}/burgers1d_sparse_jacobian.py
  ${testdir1dest} COPYONLY)
configure_file(${testdir1src}/basis.txt
  ${testdir1dest} COPYONLY)

#-----------------------
# steady lspg
#-----------------------
set(testdir2src  ${CMAKE_CURRENT_SOURCE_DIR}/demo_steady_lspg)
set(testdir2dest ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/demo_steady_lspg)
file(MAKE_DIRECTORY ${testdir2dest})
configure_file(${testdir2src}/test_demo_steady_lspg.py ${testdir2dest} COPYONLY)

#-----------------------
# unsteady lspg
#-----------------------
set(testdir2src  ${CMAKE_CURRENT_SOURCE_DIR}/burgers1d_lspg)
set(testdir2dest ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/burgers1d_lspg)
file(MAKE_DIRECTORY ${testdir2dest})
# needs the Burgers1d app, the basis file and test main driver
configure_file(${testdir2src}/test_burgers1d_lspg.py
  ${testdir2dest} COPYONLY)
configure_file(${PRESSIO4PY_APPS_DIR}/burgers1d_sparse_jacobian.py
  ${testdir2dest} COPYONLY)
configure_file(${testdir2src}/svd_basis_ncell20_t010_dt001_implicit_euler.txt
  ${testdir2dest} COPYONLY)

#-----------------------
# unsteady lspg lm solver
#-----------------------
set(testdir2src  ${CMAKE_CURRENT_SOURCE_DIR}/burgers1d_lspg_lm)
set(testdir2dest ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/burgers1d_lspg_lm)
file(MAKE_DIRECTORY ${testdir2dest})
configure_file(${testdir2src}/test_burgers1d_lspg_lm.py
  ${testdir2dest} COPYONLY)
configure_file(${PRESSIO4PY_APPS_DIR}/burgers1d_sparse_jacobian.py
  ${testdir2dest} COPYONLY)
configure_file(${testdir2src}/svd_basis_ncell20_t010_dt001_implicit_euler.txt
  ${testdir2dest} COPYONLY)

#-----------------------
# unsteady lspg custom mapping
#-----------------------
set(testdir2src  ${CMAKE_CURRENT_SOURCE_DIR}/burgers1d_lspg_custom_mapping)
set(testdir2dest ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/burgers1d_lspg_custom_mapping)
file(MAKE_DIRECTORY ${testdir2dest})
# needs the Burgers1d app, the basis file and test main driver
configure_file(${testdir2src}/test_burgers1d_lspg_cm.py
  ${testdir2dest} COPYONLY)
configure_file(${PRESSIO4PY_APPS_DIR}/burgers1d_sparse_jacobian.py
  ${testdir2dest} COPYONLY)
configure_file(${testdir2src}/svd_basis_ncell20_t010_dt001_implicit_euler.txt
  ${testdir2dest} COPYONLY)

#-----------------------
# unsteady hypred lspg
#-----------------------
set(testdir2src  ${CMAKE_CURRENT_SOURCE_DIR}/unsteady_hyperreduced_lspg)
set(testdir2dest ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/unsteady_hyperreduced_lspg)
file(MAKE_DIRECTORY ${testdir2dest})
configure_file(${testdir2src}/test_unsteady_hyp_red_lspg.py ${testdir2dest} COPYONLY)
