file(REMOVE_RECURSE
  "gimbal_controller_parameters/include/gimbal_controller_parameters.hpp"
  "gimbal_controller_parameters/include/validate_gimbal_controller_parameters.hpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/gimbal_controller_parameters.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
