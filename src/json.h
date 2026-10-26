// Minimal JSON parser covering the subset used by setup.json:
// objects, arrays, numbers, strings, booleans, null. Header-only.
#pragma once

#include <cctype>
#include <cstdlib>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace minjson {

class Value;
using Array = std::vector<Value>;
using Object = std::map<std::string, Value>;

class Value {
public:
    enum Type { NUL, BOOL, NUM, STR, ARR, OBJ };

    Type type = NUL;
    bool boolean = false;
    double number = 0.0;
    std::string string;
    std::shared_ptr<Array> array;
    std::shared_ptr<Object> object;

    bool isNull() const { return type == NUL; }
    bool isBool() const { return type == BOOL; }
    bool isNumber() const { return type == NUM; }
    bool isString() const { return type == STR; }
    bool isArray() const { return type == ARR; }
    bool isObject() const { return type == OBJ; }

    const Value& at(const std::string& key) const {
        if (type != OBJ) throw std::runtime_error("json: expected object");
        auto it = object->find(key);
        if (it == object->end()) throw std::runtime_error("json: missing key '" + key + "'");
        return it->second;
    }

    const Value& at(size_t i) const {
        if (type != ARR) throw std::runtime_error("json: expected array");
        if (i >= array->size()) throw std::runtime_error("json: index out of range");
        return (*array)[i];
    }

    bool has(const std::string& key) const {
        return type == OBJ && object->count(key) > 0;
    }

    size_t size() const {
        if (type == ARR) return array->size();
        if (type == OBJ) return object->size();
        throw std::runtime_error("json: expected array or object");
    }
};

class Parser {
public:
    explicit Parser(const std::string& text) : text_(text) {}

    Value parse() {
        skipWs();
        Value v = parseValue();
        skipWs();
        if (pos_ != text_.size()) fail("trailing characters");
        return v;
    }

private:
    [[noreturn]] void fail(const std::string& msg) const {
        std::ostringstream os;
        os << "json parse error at offset " << pos_ << ": " << msg;
        throw std::runtime_error(os.str());
    }

    void skipWs() {
        while (pos_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[pos_]))) ++pos_;
    }

    char peek() {
        if (pos_ >= text_.size()) fail("unexpected end of input");
        return text_[pos_];
    }

    void expect(char c) {
        if (pos_ >= text_.size() || text_[pos_] != c) {
            std::ostringstream os;
            os << "expected '" << c << "'";
            fail(os.str());
        }
        ++pos_;
    }

    bool literal(const char* lit) {
        size_t n = std::string(lit).size();
        if (text_.compare(pos_, n, lit) == 0) {
            pos_ += n;
            return true;
        }
        return false;
    }

    Value parseValue() {
        skipWs();
        char c = peek();
        switch (c) {
            case '{': return parseObject();
            case '[': return parseArray();
            case '"': return parseString();
            case 't': case 'f': return parseBool();
            case 'n': return parseNull();
            default: return parseNumber();
        }
    }

    Value parseObject() {
        Value v;
        v.type = Value::OBJ;
        v.object = std::make_shared<Object>();
        expect('{');
        skipWs();
        if (peek() == '}') { ++pos_; return v; }
        while (true) {
            skipWs();
            std::string key = parseString().string;
            skipWs();
            expect(':');
            v.object->emplace(key, parseValue());
            skipWs();
            char c = peek();
            if (c == ',') { ++pos_; continue; }
            if (c == '}') { ++pos_; return v; }
            fail("expected ',' or '}'");
        }
    }

    Value parseArray() {
        Value v;
        v.type = Value::ARR;
        v.array = std::make_shared<Array>();
        expect('[');
        skipWs();
        if (peek() == ']') { ++pos_; return v; }
        while (true) {
            v.array->push_back(parseValue());
            skipWs();
            char c = peek();
            if (c == ',') { ++pos_; continue; }
            if (c == ']') { ++pos_; return v; }
            fail("expected ',' or ']'");
        }
    }

    Value parseString() {
        Value v;
        v.type = Value::STR;
        expect('"');
        while (true) {
            if (pos_ >= text_.size()) fail("unterminated string");
            char c = text_[pos_++];
            if (c == '"') return v;
            if (c == '\\') {
                if (pos_ >= text_.size()) fail("unterminated escape");
                char e = text_[pos_++];
                switch (e) {
                    case '"': v.string += '"'; break;
                    case '\\': v.string += '\\'; break;
                    case '/': v.string += '/'; break;
                    case 'n': v.string += '\n'; break;
                    case 't': v.string += '\t'; break;
                    case 'r': v.string += '\r'; break;
                    default: fail("unsupported escape");
                }
            } else {
                v.string += c;
            }
        }
    }

    Value parseBool() {
        Value v;
        v.type = Value::BOOL;
        if (literal("true")) { v.boolean = true; return v; }
        if (literal("false")) { v.boolean = false; return v; }
        fail("invalid literal");
    }

    Value parseNull() {
        Value v;
        if (literal("null")) return v;
        fail("invalid literal");
    }

    Value parseNumber() {
        char* end = nullptr;
        double d = std::strtod(text_.c_str() + pos_, &end);
        if (end == text_.c_str() + pos_) fail("invalid number");
        pos_ = static_cast<size_t>(end - text_.c_str());
        Value v;
        v.type = Value::NUM;
        v.number = d;
        return v;
    }

    const std::string& text_;
    size_t pos_ = 0;
};

inline Value parse(const std::string& text) { return Parser(text).parse(); }

}  // namespace minjson
