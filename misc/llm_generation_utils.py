import re 

def post_process_llama3_program(inst):
    if inst.find("<|start_header_id|>assistant<|end_header_id|>") != -1:
        inst = inst.replace("<|start_header_id|>assistant<|end_header_id|>", "").strip()
    if inst.find("```python") != -1:
        inst = inst[inst.find("```python") + len("```python"):]
    if inst.find("```") != -1:
        inst = inst[:inst.find("```")]
    return inst

def truncate_code_at_stopwords(code):
    start_length = len("def task_program():\n")
    start_idx = code.find("def task_program():\n")
    parse_code = code[start_idx+start_length:]
    min_stop_idx = len(parse_code)
    if start_idx == -1:
        return code
    
    pattern = r'\n[^\s\n]+' # anything that escapes the function
    match = re.search(pattern, parse_code)
    if match:
        min_stop_idx = re.search(pattern, parse_code).start()
    return code[start_idx:min_stop_idx + start_idx+start_length]

def post_process_vllm_generation(outputs):
    programs = []
    for output in outputs:
        program = output.outputs[0].text
        program = post_process_llama3_program(program)
        if "def task_program():\n" not in program:
            program = "def task_program():\n" + program
        program = truncate_code_at_stopwords(program)
        programs.append(program)
    return programs