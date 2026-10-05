#include "db/Sqlite.h"

#include "core/Platform.h"

#include <sqlite3.h>

#include <chrono>
#include <thread>

namespace occlusa::db::sqlite {

Database::Database(const std::filesystem::path& path)
{
    open(path);
}

Database::~Database()
{
    close();
}

void Database::open(const std::filesystem::path& path)
{
    close();
    const std::string utf8 = platform::pathToUtf8(path);
    const int rc = sqlite3_open_v2(utf8.c_str(), &db_, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX, nullptr);
    if (rc != SQLITE_OK) {
        std::string msg = db_ ? sqlite3_errmsg(db_) : "out of memory";
        close();
        throw DbError("Cannot open database " + utf8 + ": " + msg);
    }
    sqlite3_extended_result_codes(db_, 1);
    // Wait for other writers instead of failing immediately (the file is shared between workstations).
    sqlite3_busy_timeout(db_, 15000);
}

void Database::close()
{
    if (db_) {
        sqlite3_close_v2(db_);
        db_ = nullptr;
    }
}

void Database::throwError(const std::string& context) const
{
    const int code = db_ ? sqlite3_extended_errcode(db_) : SQLITE_ERROR;
    const std::string msg = db_ ? sqlite3_errmsg(db_) : "database not open";
    if ((code & 0xFF) == SQLITE_BUSY || (code & 0xFF) == SQLITE_LOCKED)
        throw DbError(context + ": the case database is busy (another workstation is writing). Please retry.");
    throw DbError(context + ": " + msg);
}

void Database::exec(const std::string& sql)
{
    char* err = nullptr;
    if (sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &err) != SQLITE_OK) {
        std::string msg = err ? err : "unknown error";
        sqlite3_free(err);
        throwError("SQL error (" + msg + ")");
    }
}

Statement Database::prepare(const std::string& sql)
{
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql.c_str(), static_cast<int>(sql.size()), &stmt, nullptr) != SQLITE_OK)
        throwError("Prepare failed for \"" + sql.substr(0, 60) + "\"");
    return Statement(*this, stmt);
}

std::int64_t Database::lastInsertId() const
{
    return sqlite3_last_insert_rowid(db_);
}

int Database::changes() const
{
    return sqlite3_changes(db_);
}

int Database::userVersion()
{
    auto st = prepare("PRAGMA user_version");
    st.step();
    return static_cast<int>(st.getInt(0));
}

void Database::setUserVersion(int v)
{
    exec("PRAGMA user_version = " + std::to_string(v));
}

Statement::~Statement()
{
    if (stmt_)
        sqlite3_finalize(stmt_);
}

Statement& Statement::bind(int index, std::int64_t v)
{
    if (sqlite3_bind_int64(stmt_, index, v) != SQLITE_OK)
        db_->throwError("bind");
    return *this;
}

Statement& Statement::bind(int index, double v)
{
    if (sqlite3_bind_double(stmt_, index, v) != SQLITE_OK)
        db_->throwError("bind");
    return *this;
}

Statement& Statement::bind(int index, const std::string& v)
{
    if (sqlite3_bind_text(stmt_, index, v.c_str(), static_cast<int>(v.size()), SQLITE_TRANSIENT) != SQLITE_OK)
        db_->throwError("bind");
    return *this;
}

Statement& Statement::bindNull(int index)
{
    if (sqlite3_bind_null(stmt_, index) != SQLITE_OK)
        db_->throwError("bind");
    return *this;
}

bool Statement::step()
{
    const int rc = sqlite3_step(stmt_);
    if (rc == SQLITE_ROW)
        return true;
    if (rc == SQLITE_DONE)
        return false;
    db_->throwError("Query failed");
}

void Statement::run()
{
    while (step()) {
    }
}

void Statement::reset()
{
    sqlite3_reset(stmt_);
    sqlite3_clear_bindings(stmt_);
}

std::int64_t Statement::getInt(int col) const
{
    return sqlite3_column_int64(stmt_, col);
}

double Statement::getDouble(int col) const
{
    return sqlite3_column_double(stmt_, col);
}

std::string Statement::getText(int col) const
{
    const auto* p = sqlite3_column_text(stmt_, col);
    if (!p)
        return {};
    return std::string(reinterpret_cast<const char*>(p), static_cast<std::size_t>(sqlite3_column_bytes(stmt_, col)));
}

bool Statement::isNull(int col) const
{
    return sqlite3_column_type(stmt_, col) == SQLITE_NULL;
}

Transaction::Transaction(Database& db) : db_(db)
{
    db_.exec("BEGIN IMMEDIATE");
}

Transaction::~Transaction()
{
    if (!done_) {
        try {
            db_.exec("ROLLBACK");
        } catch (...) {
        }
    }
}

void Transaction::commit()
{
    db_.exec("COMMIT");
    done_ = true;
}

} // namespace occlusa::db::sqlite
