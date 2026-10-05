#pragma once

#include "db/CaseRepository.h"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

struct sqlite3;
struct sqlite3_stmt;

namespace occlusa::db::sqlite {

class Statement;

// Thin RAII wrapper around a SQLite connection.
class Database {
public:
    Database() = default;
    explicit Database(const std::filesystem::path& path);
    ~Database();
    Database(const Database&) = delete;
    Database& operator=(const Database&) = delete;

    void open(const std::filesystem::path& path);
    void close();
    bool isOpen() const { return db_ != nullptr; }

    void exec(const std::string& sql);
    Statement prepare(const std::string& sql);
    std::int64_t lastInsertId() const;
    int changes() const;
    int userVersion();
    void setUserVersion(int v);

    sqlite3* handle() const { return db_; }
    [[noreturn]] void throwError(const std::string& context) const;

private:
    sqlite3* db_ = nullptr;
};

class Statement {
public:
    Statement(Database& db, sqlite3_stmt* stmt) : db_(&db), stmt_(stmt) {}
    ~Statement();
    Statement(Statement&& o) noexcept : db_(o.db_), stmt_(o.stmt_) { o.stmt_ = nullptr; }
    Statement(const Statement&) = delete;
    Statement& operator=(const Statement&) = delete;

    Statement& bind(int index, std::int64_t v);
    Statement& bind(int index, int v) { return bind(index, static_cast<std::int64_t>(v)); }
    Statement& bind(int index, double v);
    Statement& bind(int index, const std::string& v);
    Statement& bind(int index, const char* v) { return bind(index, std::string(v)); }
    Statement& bindNull(int index);

    // Returns true while rows are available.
    bool step();
    void run(); // step to completion, for statements without results
    void reset();

    std::int64_t getInt(int col) const;
    double getDouble(int col) const;
    std::string getText(int col) const;
    bool isNull(int col) const;

private:
    Database* db_;
    sqlite3_stmt* stmt_;
};

// BEGIN IMMEDIATE ... COMMIT, rolled back if not committed. IMMEDIATE takes the write lock
// up front, which avoids deadlocks between concurrent writers on a shared database file.
class Transaction {
public:
    explicit Transaction(Database& db);
    ~Transaction();
    void commit();

private:
    Database& db_;
    bool done_ = false;
};

} // namespace occlusa::db::sqlite
