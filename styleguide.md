# Styleguide

Use the PEP8 guidelines to write your code. There are some basic PEP8 rules like using `snake_case` naming style for variables, functions, filenames and `CapWords` for classes, make sure to follow them.

[This](https://realpython.com/python-pep8/) is a good reference document/article on how to use the PEP8 rules

### How to format your code?
1. We are going to use the `black` formatter. Install it by
```bash
pip install black
```
2. Run black using
```bash
black source_dir/filename
```
3. Black should automatically format all the files.

## How to check if you are following your rules?

1. First install `pylint`
```bash
pip install pylint
```
2. Then run `pylint` on changed files
```bash
pylint filename1.py filename2.py ...
```
3. Follow the recommendations, unless its an obvious mistake.
