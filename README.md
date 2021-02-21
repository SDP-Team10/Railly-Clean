# Railly-Clean

1. During development, switch to the branch for the [current milestone](https://github.com/SDP-Team10/Railly-Clean/tree/table-sanitisation) before pushing files to the repo.
2. When committing changes, try to reference relevant open issues by including `#X` in your commit message where X is the reference number of the issue.
```bash
git commit -m "Add basic movement functions #2"
```
```bash
git commit -m "#2 Add basic movement functions"
```
3. For integration's sakes, it is recommended to represent functionalities as classes and functions residing in `libraries/`.
4. Often times, the controller will need to pass its `Robot` instance to the functionality classes and functions it calls.
5. After integration, files no longer needed for the milestone should be moved to `archive/`.
6. After testing completes, the milestone branch will be merged into `main`.
7. The file structure tries to emulate a [Webots project directory](https://cyberbotics.com/doc/guide/the-standard-file-hierarchy-of-a-project). Python scripts are either in `controllers/` or `libraries/`.
8. Webots [automatically detects controllers](https://cyberbotics.com/doc/guide/controller-start-up) if they follow the following structure `controllers/xyz_controller/xyz_controller.py`.
9. It is recommended to follow [PEP 8](https://www.python.org/dev/peps/pep-0008/) style guide.
