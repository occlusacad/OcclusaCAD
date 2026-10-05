# occlusa_db: case database (SQLite on a network share; cloud backend stubbed).
add_library(occlusa_db STATIC
    CaseModel.cpp
    CloudCaseRepository.cpp
    LocalFileStore.cpp
    RepositoryFactory.cpp
    Sqlite.cpp
    SqliteCaseRepository.cpp
)
target_include_directories(occlusa_db PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(occlusa_db PUBLIC occlusa_core PRIVATE occlusa_sqlite3)
occlusa_set_compile_options(occlusa_db)
