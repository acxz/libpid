# Collect example files
file(GLOB examples "*.cpp")
list(SORT examples)
foreach(example_file ${examples})
	get_filename_component(example_base ${example_file} NAME_WE)
    message(STATUS "Adding ${example_base} example file")

    add_executable(${example_base} ${example_file})

    target_link_libraries(${example_base})
    # TODO: what and why is below
    add_custom_target(${example_base}.run ${example_base} ${ARGN})
endforeach(example_file)
