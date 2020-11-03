
#-----------------------------
# One command to rule them all
#-----------------------------
# for this to work correctly, the names of the main python script
# in each regressions test folders MUST be unique

# Get all reg test directories and copy them, along with their contents
file(GLOB REG_TEST_FOLDERS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*)
message("Source DIR for reg test : " ${REG_TEST_FOLDERS_SRC})
file(COPY ${REG_TEST_FOLDERS_SRC} DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

# Cops the Apps directory as well
file(COPY ${PRESSIO4PY_APPS_DIR} DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
