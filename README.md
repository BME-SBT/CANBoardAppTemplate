# CAN Board App template repo

This example project serves as a template for app development targeting the HEF CAN Boards.

## Create a new app

1. Clone this repo (--recuresive required for the core library):
```bash
git clone https://github.com/BME-SBT/CANBoardAppTemplate --recursive <App folder name>
```
2. Rename origin remote
```bash
git remote rename origin upstream
```
3. Add app repo as origin
```bash
git remote add origin <git remote url here>
```
4. Track master branch
```bash
git push -u origin master
```

## Updating an existing app
1. Checkout master branch
```bash
git checkout master
```
2. Fetch upstream changes
```bash
git fetch upstream
```
3. Merge upstream changes
```bash
git merge upstream/master
```

## Update core-lib submodule
1. Update submodule
```bash
git submodule update --remote
```