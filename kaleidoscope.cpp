#include <string>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>
#include <map>

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

/* =======
 * Parsing
 * =======
 */

static int CurTok;
static int getNextToken() {
    return CurTok = gettok();
}

// utils
std::unique_ptr<ExprAST> LogError(const char *str) {
    fprintf(stderr, "LogError: %s\n", str);
    return nullptr;
}

std::unique_ptr<PrototypeAST> LogErrorP(const char *str) {
    LogError(str);
    return nullptr;
}

static std::unique_ptr<ExprAST> ParseExpression();

// numberexpr ::= number
static std::unique_ptr<ExprAST> ParseNumberExpr() {
    auto result = std::unique_ptr<NumberExprAST>(new NumberExprAST(NumVal));
    getNextToken();
    return std::move(result);
};

// parenexpr ::= '(' expression ')'
static std::unique_ptr<ExprAST> ParseParenExpr() {
    getNextToken(); // eat (.
    auto v = ParseExpression();
    if (!v)
        return nullptr;

    if (CurTok != ')')
        return LogError("expected ')'");
    getNextToken(); // eat ).
    return v;
}

// identifierexpr
//   ::= identifier
//   ::= identifier '(' expression* ')'
static std::unique_ptr<ExprAST> ParseIndentifierExpr() {
    std::string idName = IdentifierStr;

    getNextToken(); // eat identifier

    if (CurTok != '(')
        return std::unique_ptr<VariableExprAST>(new VariableExprAST(idName));

    getNextToken();  // eat (
    std::vector<std::unique_ptr<ExprAST>> args;
    if (CurTok != ')') {
        while (true) {
            if (auto Arg = ParseExpression())
                args.push_back(std::move(Arg));
            else
                return nullptr;

            if (CurTok == ')')
                break;

            if (CurTok != ',')
                return LogError("Expected ')' or ',' in argument list");
            getNextToken();
        }
    }

    // Eat the ')'.
    getNextToken();

    return std::unique_ptr<CallExprAST>(new CallExprAST(idName, std::move(args)));
}

/// primary
///   ::= identifierexpr
///   ::= numberexpr
///   ::= parenexpr
static std::unique_ptr<ExprAST> ParsePrimary() {
    switch (CurTok) {
    case tok_identifier:
        return ParseIndentifierExpr();
    case tok_number:
        return ParseNumberExpr();
    case '(':
        return ParseParenExpr();
    default:
        return LogError("Unkonw token when expecting a expressions");
    }
}

static std::map<char, int> BinopPrecedence;

static int GetTokenPrecedence() {
    if (!isascii(CurTok)) return -1;

    int p = BinopPrecedence[CurTok];
    if (p <= 0) return -1;
    return p;
}

// binoprhs
//   ::= ('+' primary)*
static std::unique_ptr<ExprAST> ParseBinopRHS(int ExprPrec,
        std::unique_ptr<ExprAST> LHS) {
    while (true) {
        int tokPrec = GetTokenPrecedence();

        if (tokPrec < ExprPrec) return LHS;

        int BinOp = CurTok;
        getNextToken(); // eat binop

        auto rhs = ParsePrimary();
        if (!rhs) return nullptr;

        int nextPrec = GetTokenPrecedence();
        if (tokPrec < nextPrec) {
            rhs = ParseBinopRHS(nextPrec + 1, std::move(rhs));
            if (!rhs) return nullptr;
        }

        LHS = std::unique_ptr<BinaryExprAST>(
                new BinaryExprAST(BinOp, std::move(LHS), std::move(rhs)));
    }
}

// expression
//   ::= primary binoprhs
static std::unique_ptr<ExprAST> ParseExpression() {
    auto lhs = ParsePrimary();
    if (!lhs) return nullptr;
    return ParseBinopRHS(0, std::move(lhs));
}

// prototype
//   ::= id '(' id* ')'
static std::unique_ptr<PrototypeAST> ParsePrototype() {
    if (CurTok != tok_identifier)
        return LogErrorP("Expected function name in prototype");

    std::string fnName = IdentifierStr;

    getNextToken();
    if (CurTok != '(')
        return LogErrorP("Expected '(' in prototype");

    std::vector<std::string> args;
    while (getNextToken() == tok_identifier)
        args.emplace_back(IdentifierStr);

    if (CurTok != ')')
        return LogErrorP("Expected ')' in prototype");
    getNextToken();

    return std::unique_ptr<PrototypeAST>(
            new PrototypeAST(fnName, std::move(args)));
}

// definition ::= 'def' prototype expression
static std::unique_ptr<FunctionAST> ParseDefinition() {
    getNextToken(); // eat 'def'
    auto proto = ParsePrototype();
    if (!proto) return nullptr;

    if (auto e = ParseExpression())
        return std::unique_ptr<FunctionAST>(
                new FunctionAST(std::move(proto), std::move(e)));
    return nullptr;
}

// external ::= 'extern' prototype
static std::unique_ptr<PrototypeAST> ParseExtern() {
    getNextToken(); // eat 'extern'
    return ParsePrototype();
}

// toplevelexpr ::= expression
static std::unique_ptr<FunctionAST> ParseTopLevelExpr() {
    if (auto e = ParseExpression()) {
        auto anonymous = std::unique_ptr<PrototypeAST>(
                new PrototypeAST("_anonymous_", std::vector<std::string>()));
        return std::unique_ptr<FunctionAST>(
                new FunctionAST(std::move(anonymous), std::move(e)));
    }
    return nullptr;
}

/* =================================
 * main flow and top level functions
 * =================================
 */

static void HandleDefinition() {
	if (ParseDefinition()) {
		fprintf(stderr, "Parsed a function definition.\n");
	} else {
		// Skip token for error recovery.
		getNextToken();
	}
}

static void HandleExtern() {
	if (ParseExtern()) {
		fprintf(stderr, "Parsed an extern\n");
	} else {
		// Skip token for error recovery.
		getNextToken();
	}
}

static void HandleTopLevelExpression() {
	// Evaluate a top-level expression into an anonymous function.
	if (ParseTopLevelExpr()) {
		fprintf(stderr, "Parsed a top-level expr\n");
	} else {
		// Skip token for error recovery.
		getNextToken();
	}
}

// top ::= definition | external | expression | ';'
static void MainLoop() {
    while (true) {
        fprintf(stderr, "ready> ");
        switch (CurTok) {
        case tok_eof:
            return;
        case ';':
            getNextToken();
            break;
        case tok_def:
            HandleDefinition();
            break;
        case tok_extern:
            HandleExtern();
            break;
        default:
            HandleTopLevelExpression();
            break;
        }
    }
}

int main() {
    // init precedence;
    BinopPrecedence['<'] = 10;
    BinopPrecedence['+'] = 20;
    BinopPrecedence['-'] = 20;
    BinopPrecedence['*'] = 40;

    fprintf(stderr, "ready> ");
    getNextToken();

    MainLoop();

    return 0;
}
