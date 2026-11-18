idf_component_register(SRCS "pid.c" "speed_est.c"
                       INCLUDE_DIRS "include")

# No fused multiply-add contraction, so results match the host build and the Python reference.
target_compile_options(${COMPONENT_LIB} PRIVATE -ffp-contract=off)
