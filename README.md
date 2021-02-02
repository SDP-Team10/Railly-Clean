# Railly-Clean

1. During development, switch to the branch for the current milestone before pushing files to the repo.
2. When committing changes, try to reference relevant open issues by including `#X` in your commit message where X is the reference number of the issue.

        git commit -m "Added basic movement functions #2"

3. After integration, files no longer needed for the milestone should be moved to `archive` folder.
4. The milestone branch would be merged into `main` after testing completed.
5. The file structure tries to emulate the directory of a Webots project. Python scripts will mostly be in `controllers` folder.
6. It is recommended to follow PEP 8 style guide.
