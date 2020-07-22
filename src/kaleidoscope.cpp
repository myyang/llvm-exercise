#include <string>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>
#include <map>

#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Type.h"
#include "llvm/IR/Verifier.h"

#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include <algorithm>

#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Transforms/InstCombine/InstCombine.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/Scalar/GVN.h"
#include "llvm/Transforms/Utils.h"

#include "../include/KaleidoscopeJIT.h"

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
    tok_if = -6,
    tok_then = -7,
    tok_else = -8,
    tok_for = -9,
    tok_in = -10,
    tok_binary = -11,
    tok_unary = -12,
    tok_var = -13,
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
        if (IdentifierStr == "if") return tok_if;
        if (IdentifierStr == "then") return tok_then;
        if (IdentifierStr == "else") return tok_else;
        if (IdentifierStr == "for") return tok_for;
        if (IdentifierStr == "in") return tok_in;
        if (IdentifierStr == "binary") return tok_binary;
        if (IdentifierStr == "unary") return tok_unary;
        if (IdentifierStr == "var") return tok_var;
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
    virtual llvm::Value *codegen() = 0;
};


class NumberExprAST: public ExprAST {
    double val;

public:
    NumberExprAST(double Val): val(Val) {};
    llvm::Value *codegen() override;
};

class VariableExprAST: public ExprAST {
    std::string name;

public:
    VariableExprAST(const std::string &Name): name(Name) {};
    llvm::Value *codegen() override;

    const std::string &getName() const { return name; }
};

class BinaryExprAST: public ExprAST {
    char op;
    std::unique_ptr<ExprAST> lhs, rhs;

public:
    BinaryExprAST(char OP, std::unique_ptr<ExprAST> LHS,
            std::unique_ptr<ExprAST> RHS)
        : op(OP), lhs(std::move(LHS)), rhs(std::move(RHS)) {};
    llvm::Value *codegen() override;
};

class CallExprAST: public ExprAST {
    std::string callee;
    std::vector<std::unique_ptr<ExprAST>> args;

public:
    CallExprAST(const std::string &Callee,
            std::vector<std::unique_ptr<ExprAST>> Args)
    : callee(Callee), args(std::move(Args)) {}
    llvm::Value *codegen() override;
};

class IfExprAST: public ExprAST {
  std::unique_ptr<ExprAST> Cond, Then, Else;

public:
  IfExprAST(std::unique_ptr<ExprAST> Cond, std::unique_ptr<ExprAST> Then,
            std::unique_ptr<ExprAST> Else)
    : Cond(std::move(Cond)), Then(std::move(Then)), Else(std::move(Else)) {}

  llvm::Value *codegen() override;
};

class ForExprAST: public ExprAST {
    std::string varName;
    std::unique_ptr<ExprAST> start, end, step, body;

public:
    ForExprAST(const std::string &VarName, std::unique_ptr<ExprAST> Start,
             std::unique_ptr<ExprAST> End, std::unique_ptr<ExprAST> Step,
             std::unique_ptr<ExprAST> Body)
        : varName(VarName), start(std::move(Start)), end(std::move(End)),
        step(std::move(Step)), body(std::move(Body)) {}

    llvm::Value *codegen() override;
};

class UnaryExprAST: public ExprAST {
    char op;
    std::unique_ptr<ExprAST> operand;

public:
    UnaryExprAST(char Op, std::unique_ptr<ExprAST> Operand)
        : op(Op), operand(std::move(Operand)) {}

    llvm::Value *codegen() override;
};

class VarExprAST: public ExprAST {
    std::vector<std::pair<std::string, std::unique_ptr<ExprAST>>> varNames;
    std::unique_ptr<ExprAST> body;

public:
    VarExprAST(std::vector<std::pair<std::string, std::unique_ptr<ExprAST>>> VarNames,
            std::unique_ptr<ExprAST> Body)
        : varNames(std::move(VarNames)), body(std::move(Body)) {}

    llvm::Value *codegen() override;
};

class PrototypeAST {
    std::string name;
    std::vector<std::string> args;
    bool isOperator;
    unsigned precedence;

public:
    PrototypeAST(
            const std::string &Name, std::vector<std::string> Args,
            bool IsOperator = false, unsigned Prec = 0)
        : name(Name), args(std::move(Args)),
        isOperator(IsOperator), precedence(Prec) {};

    llvm::Function *codegen();
    const std::string &getName() const { return name; }

    bool isUnaryOp() const { return isOperator && args.size() == 1; }
    bool isBinaryOp() const { return isOperator && args.size() == 2; }

    char getOperatorName() const {
        assert(isUnaryOp() || isBinaryOp());
        return name[name.size()-1];
    }

    unsigned getBinaryPrecedence() const { return precedence; }
};

class FunctionAST {
    std::unique_ptr<PrototypeAST> proto;
    std::unique_ptr<ExprAST> body;

public:
    FunctionAST(std::unique_ptr<PrototypeAST> Proto, std::unique_ptr<ExprAST> Body)
        : proto(std::move(Proto)), body(std::move(Body)) {};
    llvm::Function *codegen();
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
    auto result = std::make_unique<NumberExprAST>(NumVal);
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
        return std::make_unique<VariableExprAST>(idName);

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

    return std::make_unique<CallExprAST>(idName, std::move(args));
}

// ifexpr ::= 'if' expression 'then' expression 'else' expression
static std::unique_ptr<ExprAST> ParseIfExpr() {
    getNextToken();

    auto Cond = ParseExpression();
    if (!Cond) return nullptr;

    if (CurTok != tok_then)
        LogError("expected then");
    getNextToken();

    auto Then = ParseExpression();
    if (!Then)
        return nullptr;

    if (CurTok != tok_else)
        LogError("expected else");

    getNextToken();
    auto Else = ParseExpression();
    if (!Else)
        return nullptr;

    //return nullptr;
    return std::make_unique<IfExprAST>(std::move(Cond), std::move(Then), std::move(Else));
}

// forexpr ::= 'for' identifier '=' expr ',' expr (',' expr)? 'in' expression
static std::unique_ptr<ExprAST> ParseForExpr() {
    getNextToken();

    if (CurTok != tok_identifier)
        return LogError("expected identifier after for");

    std::string IDName = IdentifierStr;
    getNextToken();

    if (CurTok != '=')
        return LogError("expected '=' after for");
    getNextToken();

    auto Start = ParseExpression();
    if (!Start) return nullptr;
    if (CurTok != ',')
        return LogError("expected ',' after for start value");
    getNextToken();

    auto End = ParseExpression();
    if (!End) return nullptr;

    std::unique_ptr<ExprAST> Step;
    if (CurTok == ',') {
        getNextToken();
        Step = ParseExpression();
        if (!Step)
            return nullptr;
    }

    if (CurTok != tok_in)
        return LogError("expected in after for");
    getNextToken();

    auto Body = ParseExpression();
    if (!Body) return nullptr;

    return std::make_unique<ForExprAST>(
            IDName, std::move(Start), std::move(End),
            std::move(Step), std::move(Body));
}

static std::unique_ptr<ExprAST> ParseVarExpr() {
    getNextToken();

    std::vector<std::pair<std::string, std::unique_ptr<ExprAST>>> VarNames;

    if (CurTok != tok_identifier)
        return LogError("expected identifier after var");

    while (1) {
        std::string Name = IdentifierStr;
        getNextToken(); // eat identifier

        std::unique_ptr<ExprAST> Init;
        if (CurTok == '=') {
            getNextToken(); // eat '='

            Init = ParseExpression();
            if (!Init) return nullptr;
        }

        VarNames.push_back(std::make_pair(Name, std::move(Init)));

        if (CurTok != ',') break;
        getNextToken(); // eat ','

        if (CurTok != tok_identifier)
            return LogError("expected identifier list after var");
    }

    if (CurTok != tok_in)
        return LogError("expected 'in' keyword after 'var'");

    getNextToken();
    auto Body = ParseExpression();
    if (!Body) return nullptr;

    return std::make_unique<VarExprAST>(std::move(VarNames), std::move(Body));
}

/// primary
///   ::= identifierexpr
///   ::= numberexpr
///   ::= parenexpr
///   ::= ifexpr
///   ::= forexpr
static std::unique_ptr<ExprAST> ParsePrimary() {
    switch (CurTok) {
    case tok_identifier:
        return ParseIndentifierExpr();
    case tok_number:
        return ParseNumberExpr();
    case '(':
        return ParseParenExpr();
    case tok_if:
        return ParseIfExpr();
    case tok_for:
        return ParseForExpr();
    case tok_var:
        return ParseVarExpr();
    default:
        return LogError("Unknown token when expecting an expression");
    }
}

// unary
//   ::= primary
//   ::= '!' unary
static std::unique_ptr<ExprAST> ParseUnary() {
    if (!isascii(CurTok) || CurTok == '(' || CurTok == ',')
        return ParsePrimary();

    int Opc = CurTok;
    getNextToken();
    if (auto Operand = ParseUnary())
        return std::make_unique<UnaryExprAST>(Opc, std::move(Operand));
    return nullptr;
}

static std::map<char, int> BinopPrecedence;

static int GetTokenPrecedence() {
    if (!isascii(CurTok)) return -1;

    int p = BinopPrecedence[CurTok];
    if (p <= 0) return -1;
    return p;
}

// binoprhs
//   ::= ('+' unary)*
static std::unique_ptr<ExprAST> ParseBinopRHS(int ExprPrec,
        std::unique_ptr<ExprAST> LHS) {
    while (true) {
        int tokPrec = GetTokenPrecedence();

        if (tokPrec < ExprPrec) return LHS;

        int BinOp = CurTok;
        getNextToken(); // eat binop

        auto rhs = ParseUnary();
        if (!rhs) return nullptr;

        int nextPrec = GetTokenPrecedence();
        if (tokPrec < nextPrec) {
            rhs = ParseBinopRHS(tokPrec + 1, std::move(rhs));
            if (!rhs) return nullptr;
        }

        LHS = std::make_unique<BinaryExprAST>(BinOp, std::move(LHS), std::move(rhs));
    }
}

// expression
//   ::= unary binoprhs
static std::unique_ptr<ExprAST> ParseExpression() {
    auto lhs = ParseUnary();
    if (!lhs) return nullptr;

    return ParseBinopRHS(0, std::move(lhs));
}

// prototype
//   ::= id '(' id* ')'
//   ::= binary LETTER number? (id, id)
//   ::= unary LETTER (id)
static std::unique_ptr<PrototypeAST> ParsePrototype() {
    std::string FnName;

    unsigned Kind = 0;  // 0 = identifier, 1 = unary, 2 = binary.
    unsigned BinaryPrecedence = 30;

    switch (CurTok) {
    default:
        return LogErrorP("Expected function name in prototype");
    case tok_identifier:
        FnName = IdentifierStr;
        Kind = 0;
        getNextToken();
        break;
    case tok_unary:
        getNextToken();
        if (!isascii(CurTok))
            return LogErrorP("Expected unary operator");
        FnName = "unary";
        FnName += (char)CurTok;
        Kind = 1;
        getNextToken();
        break;
    case tok_binary:
        getNextToken();
        if (!isascii(CurTok))
            return LogErrorP("Expected binary operator");

        FnName = "binary";
        FnName += (char)CurTok;
        Kind = 2;
        getNextToken();

        if (CurTok == tok_number) {
            if (NumVal < 1 || NumVal > 100)
                return LogErrorP("Invalid precedence: must be 1..100");
            BinaryPrecedence = (unsigned)NumVal;
            getNextToken();
        }
        break;
    }

    if (CurTok != '(')
        return LogErrorP("Expected '(' in prototype");

    std::vector<std::string> args;
    while (getNextToken() == tok_identifier)
        args.emplace_back(IdentifierStr);

    if (CurTok != ')')
        return LogErrorP("Expected ')' in prototype");
    getNextToken();

    if (Kind && args.size() != Kind)
        return LogErrorP("Invalid number of operands for operator");

    return std::make_unique<PrototypeAST>(
            FnName, std::move(args), Kind != 0, BinaryPrecedence);
}

// definition ::= 'def' prototype expression
static std::unique_ptr<FunctionAST> ParseDefinition() {
    getNextToken(); // eat 'def'
    auto proto = ParsePrototype();
    if (!proto) return nullptr;

    if (auto e = ParseExpression())
        return std::make_unique<FunctionAST>(std::move(proto), std::move(e));
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
        return std::make_unique<FunctionAST>(std::move(anonymous), std::move(e));
    }
    return nullptr;
}

/* =======
 * codegen
 * =======
 */

static llvm::LLVMContext TheContext;
static llvm::IRBuilder<> Builder(TheContext);
static std::unique_ptr<llvm::Module> TheModule;
static std::map<std::string, llvm::AllocaInst *> NamedValues;
static std::unique_ptr<llvm::legacy::FunctionPassManager> TheFPM;
static std::unique_ptr<llvm::orc::KaleidoscopeJIT> TheJIT;
static std::map<std::string, std::unique_ptr<PrototypeAST>> FunctionProtos;

llvm::Value *LogErrorV(const char *Str) {
    LogError(Str);
    return nullptr;
}

llvm::Value *NumberExprAST::codegen() {
    return llvm::ConstantFP::get(TheContext, llvm::APFloat(val));
}

llvm::Value *VariableExprAST::codegen() {
    llvm::Value *v = NamedValues[name];
    if (!v) LogErrorV("Unknown variable name");

    return Builder.CreateLoad(v, name.c_str());
}

llvm::Function *getFunction(std::string Name) {
    if (auto *F = TheModule->getFunction(Name))
        return F;

    auto FI = FunctionProtos.find(Name);
    if (FI != FunctionProtos.end())
        return FI->second->codegen();

    return nullptr;
}

static llvm::AllocaInst* CreateEntryBlockAlloca(
        llvm::Function *TheFunction, const std::string &VarName) {
    llvm::IRBuilder<> TmpB(&TheFunction->getEntryBlock(),
            TheFunction->getEntryBlock().begin());
    return TmpB.CreateAlloca(llvm::Type::getDoubleTy(TheContext), 0, VarName.c_str());
}


llvm::Value *BinaryExprAST::codegen() {
    if (op == '=') {
        VariableExprAST *LHSE = dynamic_cast<VariableExprAST*>(lhs.get());
        if (!LHSE) return LogErrorV("destination of '=' must be a variable");

        llvm::Value *Val = rhs->codegen();
        if (!Val) return nullptr;

        llvm::Value *Variable = NamedValues[LHSE->getName()];
        if (!Variable) return LogErrorV("Unknow variable name");

        Builder.CreateStore(Val, Variable);
        return Val;
    }

    llvm::Value *L = lhs->codegen();
    llvm::Value *R = rhs->codegen();
    if (!L || !R) return nullptr;

    switch (op) {
    case '+':
        return Builder.CreateFAdd(L, R, "addtmp");
    case '-':
        return Builder.CreateFSub(L, R, "subtmp");
    case '*':
        return Builder.CreateFMul(L, R, "multmp");
    case '<':
        L = Builder.CreateFCmpULT(L, R, "cmptmp");
        return Builder.CreateUIToFP(L, llvm::Type::getDoubleTy(TheContext), "booltmp");
    default:
        break;
    }

    std::string fnName = "binary";
    fnName += op;   // = fnName.push_back(op);
    llvm::Function *F = getFunction(fnName);
    assert(F && "binary operator not found!");

    llvm::Value *Ops[2] = {L, R};
    return Builder.CreateCall(F, Ops, "binop");
}

llvm::Value *CallExprAST::codegen() {
    llvm::Function *CalleeF = getFunction(callee);
    if (!CalleeF) return nullptr;

    if (CalleeF->arg_size() != args.size()) return LogErrorV("incorrect # args passwd");

    std::vector<llvm::Value *> ArgsV;
    for (unsigned i = 0, e = args.size(); i != e; ++i) {
        ArgsV.emplace_back(args[i]->codegen());
        if (!ArgsV.back()) return nullptr;
    }

    return Builder.CreateCall(CalleeF, ArgsV, "calltmp");
}

llvm::Value *IfExprAST::codegen() {
    llvm::Value *CondV = Cond->codegen();
    if (!CondV) return nullptr;

    CondV = Builder.CreateFCmpONE(CondV, llvm::ConstantFP::get(TheContext, llvm::APFloat(0.0)), "ifcond");

    llvm::Function *TheFunction = Builder.GetInsertBlock()->getParent();

    llvm::BasicBlock *ThenBB = llvm::BasicBlock::Create(TheContext, "then", TheFunction);
    llvm::BasicBlock *ElseBB = llvm::BasicBlock::Create(TheContext, "else");
    llvm::BasicBlock *MergeBB = llvm::BasicBlock::Create(TheContext, "ifcond");

    Builder.CreateCondBr(CondV, ThenBB, ElseBB);
    Builder.SetInsertPoint(ThenBB);
    llvm::Value *ThenV = Then->codegen();
    if (!ThenV) return nullptr;

    Builder.CreateBr(MergeBB);
    ThenBB = Builder.GetInsertBlock();

    TheFunction->getBasicBlockList().push_back(ElseBB);
    Builder.SetInsertPoint(ElseBB);

    llvm::Value *ElseV = Else->codegen();
    if (!ElseV) return nullptr;

    Builder.CreateBr(MergeBB);
    ElseBB = Builder.GetInsertBlock();

    TheFunction->getBasicBlockList().push_back(MergeBB);
    Builder.SetInsertPoint(MergeBB);

    llvm::PHINode *PN = Builder.CreatePHI(llvm::Type::getDoubleTy(TheContext), 2, "iftmp");

    PN->addIncoming(ThenV, ThenBB);
    PN->addIncoming(ElseV, ElseBB);

    return PN;
}

// Output for-loop as:
//   ...
//   start = startexpr
//   goto loop
// loop:
//   variable = phi [start, loopheader], [nextvariable, loopend]
//   ...
//   bodyexpr
//   ...
// loopend:
//   step = stepexpr
//   nextvariable = variable + step
//   endcond = endexpr
//   br endcond, loop, endloop
// outloop:
llvm::Value *ForExprAST::codegen() {
    llvm::Value *StartVal = start->codegen();
    if (!StartVal) return nullptr;

    llvm::Function *TheFunction = Builder.GetInsertBlock()->getParent();

    llvm::AllocaInst* Alloca = CreateEntryBlockAlloca(TheFunction, varName);

    Builder.CreateStore(StartVal, Alloca);

    llvm::BasicBlock *LoopBB = llvm::BasicBlock::Create(TheContext, "loop", TheFunction);

    Builder.CreateBr(LoopBB);

    Builder.SetInsertPoint(LoopBB);

    llvm::AllocaInst *OldVal = NamedValues[varName];
    NamedValues[varName] = Alloca;
    if (!body->codegen()) return nullptr;

    llvm::Value *StepVal = nullptr;
    if (step) {
        StepVal = step->codegen();
        if (!StepVal) return nullptr;
    } else {
        StepVal =
            llvm::ConstantFP::get(TheContext, llvm::APFloat(1.0));
    }

    llvm::Value *EndCond = end->codegen();
    if (!EndCond) return nullptr;

    llvm::Value *CurVal = Builder.CreateLoad(Alloca);
    llvm::Value *NextVal = Builder.CreateFAdd(CurVal, StepVal, "nextvar");
    Builder.CreateStore(NextVal, Alloca);

    EndCond = Builder.CreateFCmpONE(
            EndCond, llvm::ConstantFP::get(TheContext, llvm::APFloat(1.0)), "loopcond");

    llvm::BasicBlock *AfterBB =
        llvm::BasicBlock::Create(TheContext, "afterloop", TheFunction);

    Builder.CreateCondBr(EndCond, LoopBB, AfterBB);

    Builder.SetInsertPoint(AfterBB);

    if (OldVal) NamedValues[varName] = OldVal;
    else NamedValues.erase(varName);

    return llvm::Constant::getNullValue(
            llvm::Type::getDoubleTy(TheContext));
}

llvm::Value *UnaryExprAST::codegen() {
    llvm::Value *OperandV = operand->codegen();
    if (!OperandV)
        return nullptr;

    std::string fnName = "unary";
    fnName += op;
    llvm::Function *F = getFunction(fnName);
    if (!F) {
        return LogErrorV("Unknown unary operator");
    }
    return Builder.CreateCall(F, OperandV, "unop");
}

llvm::Value *VarExprAST::codegen() {
    std::vector<llvm::AllocaInst *> OldBindings;

    llvm::Function *TheFunction = Builder.GetInsertBlock()->getParent();

    for (unsigned i = 0, e = varNames.size(); i != e; ++i)  {
        const std::string &VarName = varNames[i].first;
        ExprAST *Init = varNames[i].second.get();
        // Emit the initializer before adding the variable to scope, this prevents
        //  the initializer from referencing the variable itself, and permits stuff
        //  like this:
        //      var a = 1 in
        //          var a = a in ...   # refers to outer 'a'.
        llvm::Value *InitVal;
        if (Init) {
            InitVal = Init->codegen();
            if (!InitVal)
                return nullptr;
        } else {
            InitVal = llvm::ConstantFP::get(TheContext, llvm::APFloat(0.0));
        }

        llvm::AllocaInst *Alloca = CreateEntryBlockAlloca(TheFunction, VarName);
        Builder.CreateStore(InitVal, Alloca);

        OldBindings.push_back(NamedValues[VarName]);

        NamedValues[VarName] = Alloca;
    }

    llvm::Value *BodyVal = body->codegen();
    if (!BodyVal) return nullptr;

    for (unsigned i = 0, e = varNames.size(); i != e; ++i)
        NamedValues[varNames[i].first] = OldBindings[i];

    return BodyVal;
}

llvm::Function *PrototypeAST::codegen() {
    std::vector<llvm::Type *> Doubles(args.size(), llvm::Type::getDoubleTy(TheContext));

    llvm::FunctionType *FT = llvm::FunctionType::get(
            llvm::Type::getDoubleTy(TheContext), Doubles, false);
    llvm::Function *F = llvm::Function::Create(
            FT, llvm::Function::ExternalLinkage, name, TheModule.get());

    unsigned idx = 0;
    for (auto &arg : F->args()) arg.setName(args[idx++]);

    return F;
}

llvm::Function *FunctionAST::codegen() {
    auto &P = *proto;
    FunctionProtos[proto->getName()] = std::move(proto);
    llvm::Function *TheFunction = getFunction(P.getName());
    if (!TheFunction) return nullptr;

    if (P.isBinaryOp())
        BinopPrecedence[P.getOperatorName()] = P.getBinaryPrecedence();

  	// Create a new basic block to start insertion into.
    llvm::BasicBlock *BB = llvm::BasicBlock::Create(TheContext, "entry", TheFunction);
    Builder.SetInsertPoint(BB);

  	// Record the function arguments in the NamedValues map.
    NamedValues.clear();
    for (auto &Arg : TheFunction->args()) {
        llvm::AllocaInst *Alloca = CreateEntryBlockAlloca(TheFunction, Arg.getName());
        Builder.CreateStore(&Arg, Alloca);
        NamedValues[Arg.getName()] = Alloca;
    }

    if (llvm::Value *RetVal = body->codegen()) {
        Builder.CreateRet(RetVal);
        llvm::verifyFunction(*TheFunction);
	    TheFPM->run(*TheFunction);
        return TheFunction;
    }

    TheFunction->eraseFromParent();
    return nullptr;

}

/* =================================
 * main flow and top level functions
 * =================================
 */

static void InitializeModuleAndPassManager() {
    TheModule = std::make_unique<llvm::Module>("my cool jit", TheContext);
    TheModule->setDataLayout(TheJIT->getTargetMachine().createDataLayout());

    TheFPM = std::make_unique<llvm::legacy::FunctionPassManager>(TheModule.get());
    TheFPM->add(llvm::createPromoteMemoryToRegisterPass());
    TheFPM->add(llvm::createInstructionCombiningPass());
    TheFPM->add(llvm::createReassociatePass());
    TheFPM->add(llvm::createGVNPass());
    TheFPM->add(llvm::createCFGSimplificationPass());
    TheFPM->doInitialization();
}

static void HandleDefinition() {
	if (auto FnAST = ParseDefinition()) {
        if (auto *FnIR = FnAST->codegen()) {
            fprintf(stderr, "Parsed a function definition:\n");
            FnIR->print(llvm::errs());
            fprintf(stderr, "\n");
            TheJIT->addModule(std::move(TheModule));
            InitializeModuleAndPassManager();
        }
	} else {
		// Skip token for error recovery.
		getNextToken();
	}
}

static void HandleExtern() {
	if (auto FnAST = ParseExtern()) {
        if (auto *FnIR = FnAST->codegen()) {
            fprintf(stderr, "Parsed an extern:\n");
            FnIR->print(llvm::errs());
            fprintf(stderr, "\n");
            FunctionProtos[FnAST->getName()] = std::move(FnAST);
        }
	} else {
		// Skip token for error recovery.
		getNextToken();
	}
}

static void HandleTopLevelExpression() {
	// Evaluate a top-level expression into an anonymous function.
	if (auto FnAST = ParseTopLevelExpr()) {
        if (auto *FnIR = FnAST->codegen()) {
            fprintf(stderr, "Parsed a top-level expr\n");
            FnIR->print(llvm::errs());
            fprintf(stderr, "\n");

            auto H = TheJIT->addModule(std::move(TheModule));
            InitializeModuleAndPassManager();

            auto ExprSymbol = TheJIT->findSymbol("_anonymous_");
            assert(ExprSymbol && "Function not found");

            double (*FP)() = (double (*)())(intptr_t)cantFail(ExprSymbol.getAddress());
            fprintf(stderr, "Evaluated to %f\n", FP());

            TheJIT->removeModule(H);
        }
	} else {
		// Skip token for error recovery.
		getNextToken();
	}
}

//===----------------------------------------------------------------------===//
// "Library" functions that can be "extern'd" from user code.
//===----------------------------------------------------------------------===//

#ifdef _WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif

/// putchard - putchar that takes a double and returns 0.
extern "C" DLLEXPORT double putchard(double X) {
    fputc((char)X, stderr);
    return 0;
}

/// printd - printf that takes a double prints it as "%f\n", returning 0.
extern "C" DLLEXPORT double printd(double X) {
    fprintf(stderr, "%f\n", X);
    return 0;
}

/* ====
 * main
 * ====
 */

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
	llvm::InitializeNativeTarget();
	llvm::InitializeNativeTargetAsmPrinter();
	llvm::InitializeNativeTargetAsmParser();

    // init precedence;
    BinopPrecedence['='] = 2;
    BinopPrecedence['<'] = 10;
    BinopPrecedence['+'] = 20;
    BinopPrecedence['-'] = 20;
    BinopPrecedence['*'] = 40;

    fprintf(stderr, "ready> ");
    getNextToken();

    TheJIT = std::make_unique<llvm::orc::KaleidoscopeJIT>();

    InitializeModuleAndPassManager();

    MainLoop();

    TheModule->print(llvm::errs(), nullptr);

    return 0;
}
