idf_component_register(SRCS "src/proto.c" "src/plant_model.c" "src/controller_fsm.c" "src/blackbox.c"
                       INCLUDE_DIRS "include")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
