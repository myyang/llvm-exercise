#include <string>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

/* =====
 * Lexer
 * =====
 */
enum Token {
    tok_eof = -1,

    tok_def = -2,
    tok_extern = -3,

    tok_identifier = -4,
    tok_number = -5,
};

// the parse result would be safe here through lexing.
static std::string IdentifierStr;
static double NumVal;

static int gettok() {
    static int lastChar = ' ';

    while (isspace(lastChar)) lastChar = getchar();

    // parsing identifier
    if (isalpha(lastChar)) {
        IdentifierStr = lastChar;
        while (isalnum(lastChar = getchar())) IdentifierStr += lastChar;

        if (IdentifierStr == "def") return tok_def;
        if (IdentifierStr == "extern") return tok_extern;
        return tok_identifier;
    }

    // parsing number
    if (isdigit(lastChar) || lastChar == '.') {
        std::string numRaw;
        do {
            numRaw += lastChar;
            lastChar = getchar();
        } while (isdigit(lastChar) || lastChar == '.');

        NumVal = strtod(numRaw.c_str(), nullptr);
        return tok_number;
    }

    // comment
    if (lastChar == '#') {
        do {
            lastChar = getchar();
        } while (lastChar != EOF && lastChar != '\n' && lastChar != '\r');

        if (lastChar != EOF) return gettok();
    }

    if (lastChar == EOF) return tok_eof;

    int thisChar = lastChar;
    lastChar = getchar();
    return thisChar;
}

/* ===
 * AST
 * ===
 */

class ExprAST {
public:
    virtual ~ExprAST() = default;
};


class NumberExprAST: public ExprAST {
    double val;

public:
    NumberExprAST(double Val): val(Val) {};
};

class VariableExprAST: public ExprAST {
    std::string name;

public:
    VariableExprAST(const std::string &Name): name(Name) {};
};

class BinaryExprAST: public ExprAST {
    char op;
    std::unique_ptr<ExprAST> lhs, rhs;

public:
    BinaryExprAST(char OP, std::unique_ptr<ExprAST> LHS,
            std::unique_ptr<ExprAST> RHS)
        : op(OP), lhs(std::move(LHS)), rhs(std::move(RHS)) {};
};

class CallExprAST: public ExprAST {
    std::string callee;
    std::vector<std::unique_ptr<ExprAST>> args;

public:
    CallExprAST(const std::string &Callee,
            std::vector<std::unique_ptr<ExprAST>> Args)
    : callee(Callee), args(std::move(Args)) {}
};

class PrototypeAST {
    std::string name;
    std::vector<std::string> args;

public:
    PrototypeAST(const std::string &Name, std::vector<std::string> Args)
        : name(Name), args(std::move(Args)) {};
};

class FunctionAST {
    std::unique_ptr<PrototypeAST> proto;
    std::unique_ptr<ExprAST> body;

public:
    FunctionAST(std::unique_ptr<PrototypeAST> Proto, std::unique_ptr<ExprAST> Body)
        : proto(std::move(Proto)), body(std::move(Body)) {};
};
