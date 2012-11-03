H_FILENAME = "/home/joschu/bulletsim/src/sqp/expr_ops_autogen.h"
CPP_FILENAME = "/home/joschu/bulletsim/src/sqp/expr_ops_autogen.cpp"

hfile = open(H_FILENAME, "w")
cppfile = open(CPP_FILENAME, "w")

hfile.write("""
""")
cppfile.write("""
""")

def calc_deg(ta, tb, op):
    deg = dict(QExprVector = 2, VarVector = 1, ExprVector = 1, GRBVar = 1, double = 0,
               VectorXd = 0)
    if "*" in op:
        return deg[ta] + deg[tb]
    else:
        return max(deg[ta], deg[tb])

def make_vv_binop(eng_name, symb, ta, tb):
    deg = calc_deg(ta, tb, symb)
    if deg == 1: tout = "ExprVector"
    elif deg == 2: tout = "QExprVector"
    else: return
            
    d = dict(eng_name = eng_name, symb = symb, ta=ta, tb=tb, tout=tout)
    sig = r"""%(tout)s expr%(eng_name)s(const %(ta)s& a, const %(tb)s& b)"""%d
    d["sig"] = sig
    hfile.write("%(sig)s;\n"%d)
    cppfile.write(r"""
%(sig)s {
    assert(a.size()==b.size());
    %(tout)s out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] %(symb)s b[i];
    }
    return out;
}"""%d)
def make_vv_inplace(eng_name, symb, ta, tb):

    d = dict(eng_name = eng_name, symb = symb, ta=ta, tb=tb)
    sig = r"""ExprVector expr%(eng_name)s(%(ta)s& a, const %(tb)s& b)"""%d
    d["sig"] = sig
    hfile.write("%(sig)s;\n"%d)
    cppfile.write(r"""
%(sig)s {
    assert(a.size()==b.size());
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    a[i] %(symb)s b[i];
    }
    return out;
}"""%d)

def make_vs_binop(eng_name, symb, ta, tb):
    deg = calc_deg(ta, tb, symb)
    if deg == 1: tout = "ExprVector"
    elif deg == 2: tout = "QExprVector"
    else: return

    d = dict(eng_name = eng_name, symb = symb, ta=ta, tb=tb, tout=tout)
    sig = r"""%(tout)s expr%(eng_name)s(const %(ta)s& a, const %(tb)s& b)"""%d
    d["sig"] = sig
    hfile.write("%(sig)s;\n"%d)
    cppfile.write(r"""
%(sig)s {
    %(tout)s out(a.size());
    for (int i=0; i < a.size(); ++i) {
    out[i] = a[i] %(symb)s b;
    }
    return out;
}"""%d)
def make_vs_inplace(eng_name, symb, ta, tb):
    d = dict(eng_name = eng_name, symb = symb, ta=ta, tb=tb)
    sig = r"""ExprVector expr%(eng_name)s(%(ta)s& a, const %(tb)s& b)"""%d
    d["sig"] = sig
    hfile.write("%(sig)s;\n"%d)
    cppfile.write(r"""
%(sig)s {
    ExprVector out(a.size());
    for (int i=0; i < a.size(); ++i) {
    a[i] %(symb)s b;
    }
    return out;
}"""%d)


for type1 in ["ExprVector", "VarVector", "VectorXd"]:
    for type2 in ["ExprVector", "VarVector", "VectorXd"]:
        for (opname, op) in [("Add","+"), ("Sub","-"), ("Mult","*")]:
            make_vv_binop(opname, op, type1, type2)

for type1 in ["ExprVector", "VarVector", "VectorXd"]:
    for type2 in ["double", "GRBVar"]:
        for (opname, op) in [("Add","+"), ("Sub","-"), ("Mult","*")]:
            make_vs_binop(opname, op, type1, type2)

for type1 in ["ExprVector"]:
    for type2 in ["ExprVector", "VarVector", "VectorXd"]:
        for (opname, op) in [("Inc","+="), ("Dec","-=")]:
            make_vv_inplace(opname, op, type1, type2)

for type1 in ["ExprVector"]:
    for type2 in ["double", "GRBVar"]:
        for (opname, op) in [("Inc","+="), ("Dec","-=")]:
            make_vs_inplace(opname, op, type1, type2)

hfile.close()
cppfile.close()