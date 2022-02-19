aux_source_directory(. SRC_LIST)
aux_source_directory(../Goldfarb_Optimizer GOLDFARB_OPTIMIZER_LIST)
# add_executable(force_distribution  ${SRC_LIST})
# link_directories()


pybind11_add_module(force_distribution ${SRC_LIST} ${GOLDFARB_OPTIMIZER_LIST})
# target_link_libraries(force_distribution Goldfarb_Optimizer)


