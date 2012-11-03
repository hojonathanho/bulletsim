

ExprVector exprAdd(const ExprVector& a, const ExprVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] + b[i];
    }
    return out;
}
ExprVector exprSub(const ExprVector& a, const ExprVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] - b[i];
    }
    return out;
}
QExprVector exprMult(const ExprVector& a, const ExprVector& b) {
    assert(a.size()==b.size());
    QExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] * b[i];
    }
    return out;
}
ExprVector exprAdd(const ExprVector& a, const VarVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] + b[i];
    }
    return out;
}
ExprVector exprSub(const ExprVector& a, const VarVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] - b[i];
    }
    return out;
}
QExprVector exprMult(const ExprVector& a, const VarVector& b) {
    assert(a.size()==b.size());
    QExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] * b[i];
    }
    return out;
}
ExprVector exprAdd(const ExprVector& a, const VectorXd& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] + b[i];
    }
    return out;
}
ExprVector exprSub(const ExprVector& a, const VectorXd& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] - b[i];
    }
    return out;
}
ExprVector exprMult(const ExprVector& a, const VectorXd& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] * b[i];
    }
    return out;
}
ExprVector exprAdd(const VarVector& a, const ExprVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] + b[i];
    }
    return out;
}
ExprVector exprSub(const VarVector& a, const ExprVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] - b[i];
    }
    return out;
}
QExprVector exprMult(const VarVector& a, const ExprVector& b) {
    assert(a.size()==b.size());
    QExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] * b[i];
    }
    return out;
}
ExprVector exprAdd(const VarVector& a, const VarVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] + b[i];
    }
    return out;
}
ExprVector exprSub(const VarVector& a, const VarVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] - b[i];
    }
    return out;
}
QExprVector exprMult(const VarVector& a, const VarVector& b) {
    assert(a.size()==b.size());
    QExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] * b[i];
    }
    return out;
}
ExprVector exprAdd(const VarVector& a, const VectorXd& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] + b[i];
    }
    return out;
}
ExprVector exprSub(const VarVector& a, const VectorXd& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] - b[i];
    }
    return out;
}
ExprVector exprMult(const VarVector& a, const VectorXd& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] * b[i];
    }
    return out;
}
ExprVector exprAdd(const VectorXd& a, const ExprVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] + b[i];
    }
    return out;
}
ExprVector exprSub(const VectorXd& a, const ExprVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] - b[i];
    }
    return out;
}
ExprVector exprMult(const VectorXd& a, const ExprVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] * b[i];
    }
    return out;
}
ExprVector exprAdd(const VectorXd& a, const VarVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] + b[i];
    }
    return out;
}
ExprVector exprSub(const VectorXd& a, const VarVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] - b[i];
    }
    return out;
}
ExprVector exprMult(const VectorXd& a, const VarVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] * b[i];
    }
    return out;
}
ExprVector exprAdd(const ExprVector& a, const double& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] + b;
    }
    return out;
}
ExprVector exprSub(const ExprVector& a, const double& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] - b;
    }
    return out;
}
ExprVector exprMult(const ExprVector& a, const double& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] * b;
    }
    return out;
}
ExprVector exprAdd(const ExprVector& a, const GRBVar& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] + b;
    }
    return out;
}
ExprVector exprSub(const ExprVector& a, const GRBVar& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] - b;
    }
    return out;
}
QExprVector exprMult(const ExprVector& a, const GRBVar& b) {
    QExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] * b;
    }
    return out;
}
ExprVector exprAdd(const VarVector& a, const double& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] + b;
    }
    return out;
}
ExprVector exprSub(const VarVector& a, const double& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] - b;
    }
    return out;
}
ExprVector exprMult(const VarVector& a, const double& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] * b;
    }
    return out;
}
ExprVector exprAdd(const VarVector& a, const GRBVar& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] + b;
    }
    return out;
}
ExprVector exprSub(const VarVector& a, const GRBVar& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] - b;
    }
    return out;
}
QExprVector exprMult(const VarVector& a, const GRBVar& b) {
    QExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] * b;
    }
    return out;
}
ExprVector exprAdd(const VectorXd& a, const GRBVar& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] + b;
    }
    return out;
}
ExprVector exprSub(const VectorXd& a, const GRBVar& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] - b;
    }
    return out;
}
ExprVector exprMult(const VectorXd& a, const GRBVar& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] * b;
    }
    return out;
}
ExprVector exprInc(ExprVector& a, const ExprVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    a[i] += b[i];
    }
    return out;
}
ExprVector exprDec(ExprVector& a, const ExprVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    a[i] -= b[i];
    }
    return out;
}
ExprVector exprInc(ExprVector& a, const VarVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    a[i] += b[i];
    }
    return out;
}
ExprVector exprDec(ExprVector& a, const VarVector& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    a[i] -= b[i];
    }
    return out;
}
ExprVector exprInc(ExprVector& a, const VectorXd& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    a[i] += b[i];
    }
    return out;
}
ExprVector exprDec(ExprVector& a, const VectorXd& b) {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    a[i] -= b[i];
    }
    return out;
}
ExprVector exprInc(ExprVector& a, const double& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    a[i] += b;
    }
    return out;
}
ExprVector exprDec(ExprVector& a, const double& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    a[i] -= b;
    }
    return out;
}
ExprVector exprInc(ExprVector& a, const GRBVar& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    a[i] += b;
    }
    return out;
}
ExprVector exprDec(ExprVector& a, const GRBVar& b) {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    a[i] -= b;
    }
    return out;
}