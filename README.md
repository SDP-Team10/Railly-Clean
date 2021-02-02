# Railly-Clean

1. During development, switch to the branch for the current milestone before pushing files to the repo.
2. When committing changes, try to reference relevant open issues by including `#X` in your commit message where X is the reference number of the issue.
```bash
git commit -m "Added basic movement functions #2"
```
3. After integration, files no longer needed for the milestone should be moved to `archive/`.
4. After testing completes, the milestone branch will be merged into `main`.
5. The file structure tries to emulate a Webots project directory. Python scripts will mostly be in `controllers/`.
6. It is recommended to follow [PEP 8](https://www.python.org/dev/peps/pep-0008/) style guide.
