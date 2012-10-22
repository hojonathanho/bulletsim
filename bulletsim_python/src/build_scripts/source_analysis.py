import re
def find_classes(text):
    """
    find line that contains a top-level open brace
    then look for class { in that line
    """
    nest_level = 0
    brace_re = re.compile("[\{\}]")
    class_re = re.compile("(?:class|struct)\s*(\w+)\s*(?:\:\s*public\s*\w+)?\s*\{")
    
    classes = []
    lines = text.split("\n")
    for (i,line) in enumerate(lines):
        if nest_level == 0 and (i==0 or "template" not in lines[i-1]):            
            classes.extend(class_re.findall(line))
        
        braces = brace_re.findall(line)
        for brace in braces:
            if brace == "{": nest_level += 1
            elif brace == "}": nest_level -= 1
            
    return classes