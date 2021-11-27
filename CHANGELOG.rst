^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grbl_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.16 (2021-11-27)
-------------------
* fix changelog tags that were missing dates
* bump version in prep for release
* Merge pull request `#111 <https://github.com/flynneva/grbl_ros/issues/111>`_ from flynneva/fix/default-params-and-grbl-obj-err
  Fix/default params and grbl obj err
* fix flake8 errors
* [110] add warning about missing fields in status
* set defaults for parameters, fix grbl_obj error
* add more info to quickstart README
* fixed typo
* bump ros action
* still trying to fix docs
* docs source ros
* ls ros in docs build
* remove HOME env var
* add a bunch of prints
* docs still not sourcing ros
* ugly source filepath
* source ros from opt
* full path source
* fix fpath for docs
* one more try for the docs
* dont include connext
* cd to source code before building docs
* test including connext
* forgot quotes
* test for optional connext ci
* forgot one cd
* change gh action directories
* debugging sourcing ros
* ls opt ros path to debug
* sh not bash
* try another way to source ros
* update README
* source ros with make docs cmd
* source ros before building docs
* update os for docs action
* add ros setup action to docs action
* fixed flake8 errors
* more doc updates
* more updates to docs
* update doc structure, add gitignore
* removed pytest from requirements.txt
* remove ros pkgs from requirements
* one more time
* filepath to requirements
* trying to fix filepath
* add filepath for requirements
* move dependencies install after ros env setup
* fixed yml typo
* add requirements.txt file
* switched back to ammaraskar sphinx action
* trying to update docs action
* hopefully final pep257 fix
* more pep257 fixes
* fixed pep257 errors
* Merge pull request `#75 <https://github.com/flynneva/grbl_ros/issues/75>`_ from flynneva/improve_docs
  fix flake8 errors
* fix flake8 errors
* Merge pull request `#73 <https://github.com/flynneva/grbl_ros/issues/73>`_ from flynneva/improve_docs
  Improve docs
* slowly improve docs
* remove blank line in package.xml
* bumped docs version
* forgot to bump setup.py
* Contributors: Evan Flynn, flynneva

0.0.15 (2020-10-22)
-------------------
* updated changelog spacing
* update changelog
* add extra line
* removed whitespace
* removed env var in test completely
* switched test from env var to try except
* fix pep257 error
* added geometry msgs to autodoc
* docs work, still need clean up
* trying to make docs work
* update readme & docs
* remove mkdir docs
* fixed copyright and flake8 errors
* Merge branch 'devel' of github.com:flynneva/grbl_ros into devel
* add back in docs folder
* add mkdir docs
* prepare release
* Contributors: Evan Flynn, flynneva

0.0.14 (2020-10-21)
-------------------
* update changelog
* update status badges
* update changelog
* remove docs
* bump version
* Merge branch 'main' into devel
* fixed version and removed tags
* Merge pull request `#61 <https://github.com/flynneva/grbl_ros/issues/61>`_ from flynneva/devel
  add back in ci for all ros distros
* fixed pep257
* switch to custom setup-ros branch
* fixed flake8 errors
* flake8 backwards compatability
* fixed package.xml depends
* added release actions and fixed flake8 tests
* added all ros2 versions back to ci
* Merge pull request `#58 <https://github.com/flynneva/grbl_ros/issues/58>`_ from flynneva/mixin_refactor
  refactored into mixin classes
* flake8 fixes
* reduced period per line send
* send file functioning
* send gcode cmd action functional
* functioning pose and tf pubs
* refactored into mixin classes
* progress
* beginning to parse status response
* added some more console prints
* flake8 errors
* progress
* still just getting started
* fixed some bugs
* loading in params from yaml file
* minor upgrades
* added status, pose and tf publishers
* cleaned up sending serial data
* Merge pull request `#53 <https://github.com/flynneva/grbl_ros/issues/53>`_ from flynneva/devel
  fixed logging error for stream status
* fixed logging error for stream status
* Merge pull request `#52 <https://github.com/flynneva/grbl_ros/issues/52>`_ from flynneva/devel
  added stream gcode function
* fixed flake8 errors
* added stream gcode function
* Merge pull request `#45 <https://github.com/flynneva/grbl_ros/issues/45>`_ from flynneva/devel
  moved grbl_device.py to _command.py
* moved grbl_device.py to _command.py
* Contributors: Evan Flynn, flynneva

0.0.2 (2020-08-05)
------------------
* Merge branch 'main' of github.com:flynneva/grbl_ros into main
* bump version
* Merge pull request `#7 <https://github.com/flynneva/grbl_ros/issues/7>`_ from flynneva/docs
  update readme
* update distro table
* README.md
* updated readme
* update readme
* Merge pull request `#6 <https://github.com/flynneva/grbl_ros/issues/6>`_ from flynneva/docs
  specify ros distro
* change dir name to ros_ws
* specify ros distro
* Merge pull request `#5 <https://github.com/flynneva/grbl_ros/issues/5>`_ from flynneva/docs
  add readme and fix pep257 error
* too many dashes
* add readme and fix pep257 error
* Merge pull request `#4 <https://github.com/flynneva/grbl_ros/issues/4>`_ from flynneva/docs
  add more verbose docs
* fixed lint errors
* add more verbose docs
* Merge pull request `#3 <https://github.com/flynneva/grbl_ros/issues/3>`_ from flynneva/docs
  updated docs publish dir
* updated docs publish dir
* Merge pull request `#2 <https://github.com/flynneva/grbl_ros/issues/2>`_ from flynneva/docs
  add sphinx documentation & gh pages action
* fixed lint errors
* added docs action
* add sphinx docs
* Merge pull request `#1 <https://github.com/flynneva/grbl_ros/issues/1>`_ from flynneva/actions
  added actions
* shortened comment
* removed duplicate fail-fast
* trying to get docker to work
* try to fix some bugs
* build errors
* try running ci on all OS's
* flake8 errors fixed
* ran cli ament_copyright
* fixing linting errors
* added license to each file
* trying to fix copyright tests
* added license and contributing.md
* added package name
* fixed vm to ubuntu 20.04
* trying to update virtual machine to focal 20.04
* updated to ros2 ci
* fixed some typos
* added actions
* Contributors: Evan Flynn
