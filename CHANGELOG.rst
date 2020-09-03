^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grbl_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2020-09-03)
-----------
* added stream file function
* fixed execution errors
* Merge branch 'devel' into eloquent-devel
* moved grbl_device.py to _command.py
* fixed merge conflicts for eloquent ci
* restructure code
* Merge pull request `#33 <https://github.com/flynneva/grbl_ros/issues/33>`_ from flynneva/restructure
  Restructure
* fixed flake8 errors
* added python3-serial to package.xml
* dependency is pyserial not serial
* removed old import
* setup-ros reference master branch
* switched ci to rolling for main and devel branches
* added changelog for rolling release
* separated methods to semantic file name
* Delete dashing-release.yml
* Update dashing-release.yml
* Update dashing-release.yml
* Update foxy-release-candidate.yml
* Update eloquent-release-candidate.yml
* Update dashing-release-candidate.yml
* Update dashing-release.yml
* Update and rename release.yml to dashing-release.yml
* Create foxy-release-candidate.yml
* Create eloquent-release-candidate.yml
* Update and rename ros_ci.yml to eloquent_ci.yml
* Delete release.yml
* Delete release-candidate.yml
* Merge pull request `#21 <https://github.com/flynneva/grbl_ros/issues/21>`_ from flynneva/devel
* specify distro in issue body
* Merge pull request `#18 <https://github.com/flynneva/grbl_ros/issues/18>`_ from flynneva/devel
* changed checkout branch name
* Merge pull request `#16 <https://github.com/flynneva/grbl_ros/issues/16>`_ from flynneva/devel
* renamed and specified branch to checkout
* Merge pull request `#14 <https://github.com/flynneva/grbl_ros/issues/14>`_ from flynneva/devel
* create dashing release candidate
* Update greetings.yml
* Merge pull request `#12 <https://github.com/flynneva/grbl_ros/issues/12>`_ from flynneva/devel
* rolling not valid for this setup-ros release
  specify distro in issue body
* specify distro in issue body
* Merge pull request `#18 <https://github.com/flynneva/grbl_ros/issues/18>`_ from flynneva/devel
  changed checkout branch name
* changed checkout branch name
* Merge pull request `#16 <https://github.com/flynneva/grbl_ros/issues/16>`_ from flynneva/devel
  renamed and specified branch to checkout
* renamed and specified branch to checkout
* Merge pull request `#14 <https://github.com/flynneva/grbl_ros/issues/14>`_ from flynneva/devel
  create dashing release candidate
* create dashing release candidate
* Update greetings.yml
* Merge pull request `#12 <https://github.com/flynneva/grbl_ros/issues/12>`_ from flynneva/devel
  vcs-repo-file-url param
* rolling not valid for this setup-ros release
  wait to switch to rolling until next setup-ros release
* only run if issue has eloquent in the issue
* eloquent release
* only run eloquent on eloquent branches
* dashing release candidate
* Delete greetings.yml
* Delete docs.yml
* only release dashing on dashing branch
* run on rolling for main/devel
* import main for dashing/eloquent
* remove mac from action
* switched to mac os and back to ros-tooling wg
* removed mac from matrix
* accidentally put uses on wrong step
* test action-ros-ci that sources ROS for windows
* test windows fix for ci
* vcs-repo-file-url param
* Merge pull request `#11 <https://github.com/flynneva/grbl_ros/issues/11>`_ from flynneva/devel
* removed vcs-repo-file-url param
* bump action-ros-ci & add vcs repo url
* package should not be in matrix
* Merge pull request `#10 <https://github.com/flynneva/grbl_ros/issues/10>`_ from flynneva/devel
  added ros source binary for distro
* bumped ros ci to 0.0.18
* regressed to ros ci 0.0.15
* specify target distro
* specify target distro
* added ros source binary for distro
* Merge pull request `#9 <https://github.com/flynneva/grbl_ros/issues/9>`_ from flynneva/devel
* Merge pull request `#8 <https://github.com/flynneva/grbl_ros/issues/8>`_ from flynneva/update_readme
  update readme and add release actions
* Merge pull request `#8 <https://github.com/flynneva/grbl_ros/issues/8>`_ from flynneva/update_readme
  Update readme & add release actions
* added release actions
* added testing section
* forgot to add ubuntu
* update readme
* Contributors: Evan Flynn

* separated methods to semantic file name
* Delete dashing-release.yml
* Update dashing-release.yml
* Update dashing-release.yml
* Update foxy-release-candidate.yml
* Update eloquent-release-candidate.yml
* Update dashing-release-candidate.yml
* Update dashing-release.yml
* Update and rename release.yml to dashing-release.yml
* Create foxy-release-candidate.yml
* Create eloquent-release-candidate.yml
* Merge pull request `#21 <https://github.com/flynneva/grbl_ros/issues/21>`_ from flynneva/devel
* specify distro in issue body
* Merge pull request `#18 <https://github.com/flynneva/grbl_ros/issues/18>`_ from flynneva/devel
* changed checkout branch name
* Merge pull request `#16 <https://github.com/flynneva/grbl_ros/issues/16>`_ from flynneva/devel
* renamed and specified branch to checkout
* Merge pull request `#14 <https://github.com/flynneva/grbl_ros/issues/14>`_ from flynneva/devel
* create dashing release candidate
* Update greetings.yml
* Merge pull request `#12 <https://github.com/flynneva/grbl_ros/issues/12>`_ from flynneva/devel
* rolling not valid for this setup-ros release
* run on rolling for main/devel
* remove mac from action
* switched to mac os and back to ros-tooling wg
* removed mac from matrix
* accidentally put uses on wrong step
* test action-ros-ci that sources ROS for windows
* test windows fix for ci
* vcs-repo-file-url param
* Merge pull request `#11 <https://github.com/flynneva/grbl_ros/issues/11>`_ from flynneva/devel
* removed vcs-repo-file-url param
* bump action-ros-ci & add vcs repo url
* package should not be in matrix
* Merge pull request `#10 <https://github.com/flynneva/grbl_ros/issues/10>`_ from flynneva/devel
* bumped ros ci to 0.0.18
* regressed to ros ci 0.0.15
* specify target distro
* specify target distro
* added ros source binary for distro
* Merge pull request `#9 <https://github.com/flynneva/grbl_ros/issues/9>`_ from flynneva/devel
* Merge pull request `#8 <https://github.com/flynneva/grbl_ros/issues/8>`_ from flynneva/update_readme
* added release actions
* added testing section
* forgot to add ubuntu
* update readme
* Contributors: Evan Flynn

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
