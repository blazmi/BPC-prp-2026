# 1. Hledání knihovny GTest pro testování
find_package(GTest REQUIRED)
include(GoogleTest)

# 2. Vytvoření testovacího programu
# Zde spojíme tvůj nový testovací soubor a reálnou implementaci estimátoru
add_executable(t_line_estimator
        t_line_estimator.cpp
        ${PROJECT_SOURCE_DIR}/src/algorithms/line_estimator.cpp
        t_pid.cpp
        ../src/algorithms/PlanarImuIntegrator.cpp
)

# 3. Přidání cest k hlavičkovým souborům
# Tohle říká kompilátoru, kde má hledat soubory, když v kódu napíšeš #include "..."
target_include_directories(t_line_estimator PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

# 4. Připojení knihovny GTest k našemu programu
target_link_libraries(t_line_estimator
        GTest::gtest_main
)

# 5. Registrace testu pro CLion a CTest
gtest_discover_tests(t_line_estimator)