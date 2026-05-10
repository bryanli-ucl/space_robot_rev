Import("env")

_original_Object = env.Object

def _Object_with_w_for_libs(target, source, *args, **kwargs):
    src = source
    if isinstance(source, list):
        src = source[0]
    src_path = str(src)
    project_src = env.subst("$PROJECT_DIR/src")
    
    print(f"[EXTRA_SCRIPT] Building {src_path} -> starts with src? {src_path.startswith(project_src)}")
    
    if not src_path.startswith(project_src):
        ccflags = kwargs.pop("CCFLAGS", env.get("CCFLAGS", []))
        if isinstance(ccflags, str):
            ccflags = ccflags.split()
        ccflags = list(ccflags) + ["-w"]
        kwargs["CCFLAGS"] = ccflags
        print(f"[EXTRA_SCRIPT]   -> Added -w. CCFLAGS={ccflags}")
    
    return _original_Object(target, source, *args, **kwargs)

env.Object = _Object_with_w_for_libs
