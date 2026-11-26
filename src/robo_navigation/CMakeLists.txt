cmake_minimum_required(VERSION 3.8)
project(robo_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(lateral_ws test/test_lateral_ws.py)
  ament_add_pytest_test(face_proxima test/test_face_proxima.py)
  ament_add_pytest_test(giro_frontal test/test_giro_frontal.py)
  ament_add_pytest_test(giro_integrado test/test_integrado_t.py TIMEOUT 120)
  # NAV_INTEGRADA (10/11): recuperacao avancada e arvore integrada (offline)
  ament_add_pytest_test(recuperacao_avancada test/test_recuperacao_avancada.py TIMEOUT 600)
  ament_add_pytest_test(ajustes_missao35 test/test_ajustes_missao35.py TIMEOUT 120)
  ament_add_pytest_test(supervisor_passagem test/test_supervisor_passagem.py TIMEOUT 120)
  ament_add_pytest_test(nav2_principal test/test_nav2_principal.py TIMEOUT 120)
  ament_add_pytest_test(arvore_integrada test/test_arvore_integrada.py TIMEOUT 120)
  ament_add_pytest_test(config_integrada test/test_config_integrada.py TIMEOUT 120)
  ament_add_pytest_test(destravar_ros_isolado test/test_destravar_ros_isolado.py TIMEOUT 300)
endif()

ament_package()
