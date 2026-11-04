#pragma once
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Minimal JSON reader shared by the vehicle and map loaders.
// Handles objects, arrays, strings, numbers, true/false/null — enough for the
// project's data files — and, unlike the per-loader copies it replaces, it
// reports syntax errors (truncated file, missing ':' / ',' / bracket, bad
// number) instead of silently returning a partial tree.
// ---------------------------------------------------------------------------
namespace json {

struct Value {
    enum Type { Null, Num, Bool, Str, Arr, Obj } type = Null;
    double                   num = 0.0;
    std::string              str;
    std::vector<Value>       arr;   // array items, or object values (parallel to keys)
    std::vector<std::string> keys;  // object keys

    double             asFloat()  const { return num; }
    const std::string& asString() const { return str; }
    bool               isNumber() const { return type == Num || type == Bool; }

    bool has(const std::string& k) const {
        for (const auto& key : keys) if (key == k) return true;
        return false;
    }
    const Value& at(const std::string& k) const {
        for (size_t i = 0; i < keys.size(); ++i)
            if (keys[i] == k) return arr[i];
        static const Value empty{};
        return empty;
    }
    const Value& at(size_t i) const { return arr.at(i); }
    size_t       size()       const { return arr.size(); }
};

class Parser {
public:
    explicit Parser(const std::string& text) : s_(text) {}

    // Parses the whole document; on failure returns false and error() says where.
    bool Parse(Value& out) {
        out = ParseValue();
        if (error_.empty()) {
            SkipWs();
            if (p_ != s_.size()) Fail("unexpected text after the document");
        }
        return error_.empty();
    }
    const std::string& error() const { return error_; }

private:
    const std::string& s_;
    size_t             p_ = 0;
    std::string        error_;

    void Fail(const std::string& what) {
        if (!error_.empty()) return;
        size_t line = 1;
        for (size_t i = 0; i < p_ && i < s_.size(); ++i) if (s_[i] == '\n') ++line;
        error_ = what + " (line " + std::to_string(line) + ")";
    }
    void SkipWs() { while (p_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[p_]))) ++p_; }
    bool Eat(char c) {
        SkipWs();
        if (p_ < s_.size() && s_[p_] == c) { ++p_; return true; }
        return false;
    }

    std::string ParseString() {
        std::string r;
        ++p_;  // opening quote
        while (p_ < s_.size() && s_[p_] != '"') {
            if (s_[p_] == '\\' && p_ + 1 < s_.size()) ++p_;
            r += s_[p_++];
        }
        if (p_ >= s_.size()) { Fail("unterminated string"); return r; }
        ++p_;  // closing quote
        return r;
    }

    Value ParseValue() {
        Value v;
        SkipWs();
        if (p_ >= s_.size()) { Fail("unexpected end of file"); return v; }
        const char c = s_[p_];
        if (c == '{') return ParseObject();
        if (c == '[') return ParseArray();
        if (c == '"') { v.type = Value::Str; v.str = ParseString(); return v; }
        if (s_.compare(p_, 4, "true") == 0)  { p_ += 4; v.type = Value::Bool; v.num = 1.0; return v; }
        if (s_.compare(p_, 5, "false") == 0) { p_ += 5; v.type = Value::Bool; v.num = 0.0; return v; }
        if (s_.compare(p_, 4, "null") == 0)  { p_ += 4; return v; }
        if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) {
            const char* begin = s_.c_str() + p_;
            char* end = nullptr;
            v.num = std::strtod(begin, &end);
            if (end == begin) { Fail("bad number"); return v; }
            p_ += static_cast<size_t>(end - begin);
            v.type = Value::Num;
            return v;
        }
        Fail(std::string("unexpected character '") + c + "'");
        return v;
    }

    Value ParseArray() {
        Value v; v.type = Value::Arr;
        ++p_;
        if (Eat(']')) return v;
        while (error_.empty()) {
            v.arr.push_back(ParseValue());
            if (Eat(',')) continue;
            if (Eat(']')) return v;
            Fail(p_ >= s_.size() ? "unexpected end of file in array" : "expected ',' or ']'");
        }
        return v;
    }

    Value ParseObject() {
        Value v; v.type = Value::Obj;
        ++p_;
        if (Eat('}')) return v;
        while (error_.empty()) {
            SkipWs();
            if (p_ >= s_.size() || s_[p_] != '"') { Fail(p_ >= s_.size() ? "unexpected end of file in object" : "expected a quoted key"); break; }
            std::string key = ParseString();
            if (!Eat(':')) { Fail("expected ':' after \"" + key + "\""); break; }
            v.keys.push_back(key);
            v.arr.push_back(ParseValue());
            if (Eat(',')) continue;
            if (Eat('}')) return v;
            Fail(p_ >= s_.size() ? "unexpected end of file in object" : "expected ',' or '}'");
        }
        return v;
    }
};

// Reads and parses a file. Returns false with a message in `error` when the
// file is missing or not valid JSON.
inline bool ParseFile(const std::string& path, Value& out, std::string& error) {
    std::ifstream f(path);
    if (!f.is_open()) { error = "cannot open " + path; return false; }
    std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    Parser parser(content);
    if (!parser.Parse(out)) { error = parser.error(); return false; }
    return true;
}

} // namespace json
