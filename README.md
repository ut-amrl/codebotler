Important files/folders:
 - `train/` : training data <OUTDATED> 
 - `test/` : testing data <OUTDATED>
 - `DSL.txt` : the prompt, which contains the DSL and a couple of examples
 - `train.py` : finetuning the model, based off `https://github.com/loubnabnl/santacoder-finetuning` <NOT CURRENTLY USED>
 
 Files relating to code "interpreter" are in `/robot_interpreter/`
 - `code.txt` : the code to be interpreted, in format specified by `DSL.txt`
 - `write_code.py` : the actual interpreter, converts code in `code.txt` to python
 - `robot_lib.py` : where the interpreter saves the generated code in a runnable format
 - `robot_lib_template.txt` : template for interpreting code, used by `write_code.py`
 - Usage: put the code you want to run in `code.txt`, then run `write_code.py`
