
# appraoch below is ugly... should we do this better

#-----------------------
# galerkin 
#-----------------------
set(testdir1src ${CMAKE_CURRENT_SOURCE_DIR}/burgers1d_galerkin)
set(testdir1dest ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
set(testdir1data ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/burgers1d_galerkin)

configure_file(${testdir1src}/test_burgers1d_galerkin.py ${testdir1dest} COPYONLY)
configure_file(${PRESSIO4PY_APPS_DIR}/burgers1d_sparse_jacobian.py ${testdir1dest} COPYONLY)

file(MAKE_DIRECTORY ${testdir1data})
configure_file(${testdir1src}/basis.txt ${testdir1data} COPYONLY)

#-----------------------
# lspg
#-----------------------
set(testdir2src ${CMAKE_CURRENT_SOURCE_DIR}/burgers1d_lspg)
set(testdir2dest ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
set(testdir2data ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/burgers1d_lspg)

configure_file(${testdir2src}/test_burgers1d_lspg.py ${testdir2dest} COPYONLY)
configure_file(${PRESSIO4PY_APPS_DIR}/burgers1d_sparse_jacobian.py ${testdir2dest} COPYONLY)

file(MAKE_DIRECTORY ${testdir2data})
configure_file(${testdir2src}/svd_basis_ncell20_t010_dt001_implicit_euler.txt ${testdir2data} COPYONLY)
