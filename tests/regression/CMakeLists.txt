
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
configure_file(${testdir2src}/svd_basis_ncell20_t010_dt001_implicit_bdf2.txt
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

#---------------------------
# unsteady hypred lspg bdf1
#---------------------------
set(testdir2src  ${CMAKE_CURRENT_SOURCE_DIR}/unsteady_hyperreduced_bdf1_lspg)
set(testdir2dest ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/unsteady_hyperreduced_bdf1_lspg)
file(MAKE_DIRECTORY ${testdir2dest})
configure_file(${testdir2src}/test_unsteady_hyp_red_bdf1_lspg.py ${testdir2dest} COPYONLY)

#---------------------------
# unsteady hypred lspg bdf2
#---------------------------
set(testdir2src  ${CMAKE_CURRENT_SOURCE_DIR}/unsteady_hyperreduced_bdf2_lspg)
set(testdir2dest ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/unsteady_hyperreduced_bdf2_lspg)
file(MAKE_DIRECTORY ${testdir2dest})
configure_file(${testdir2src}/test_unsteady_hyp_red_bdf2_lspg.py ${testdir2dest} COPYONLY)






# code below DOES NOT update code in build when modifying source !!!!!
# #-----------------------------
# # One command to rule them all
# #-----------------------------
# # for this to work correctly, the names of the main python script
# # in each regressions test folders MUST be unique

# # Get all reg test directories and copy them, along with their contents
# file(GLOB REG_TEST_FOLDERS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*)
# message("Source DIR for reg test : " ${REG_TEST_FOLDERS_SRC})
# file(COPY ${REG_TEST_FOLDERS_SRC} DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

# # Cops the Apps directory as well
# file(COPY ${PRESSIO4PY_APPS_DIR} DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
