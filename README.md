# ppa
Hosting a ppa allows publishing of debians that can be used in other projects.
They can then be depended on through the rosdep system, leading to a more uniform dependency system for ros projects.

Creating debians that rosdep can consume can be broken up into three major steps.

- Create a debian binary package to install
- Host the debians on a private PPA
- Update the rosdep `sources.list.d` directory

## Debian binary package
Debian binary packages are what are installed when running
```
sudo apt install package-name
```

This differs from a debian source package which also contains source code and instructions for building the binary. Creating debian source packages is not required to create debian binaries, though recommended to increase traceability and reproducibility.

A debian binary package contains
- package meta data
- the files you would like to install onto the target system
- the directories those files should be installed to

For packages that contain executables, a debian might install to
```
/usr/bin/name-of-executable
```
Which, after installed could be run with
```
$ name-of-executable
```
For C++ library packages, a debian might install to

```
/usr/include/package-name/package-header.hpp
/usr/lib/cmake/package-name/package-nameConfig.cmake
/usr/lib/cmake/package-name/package-nameTargets-none.cmake
/usr/lib/cmake/package-name/package-nameTargets.cmake
/usr/lib/libpackage-name.a
```
Which could then be linked to in your cmake project.
```
find_package(package-name REQUIRED)
target_link_libraries(target PRIVATE package-namespace::package-name)
```

### Creating a debian
`dh_make` is used to make a debian package structure.
`dpkg-buildpackage` is used to build the package.
`debclean` is useful for cleaning up after the process.
```
sudo apt install -y dh-make devscripts
```
Export name and email which is used by `dh_make`
```
export DEBEMAIL="griswald.brooks@gmail.com"
export DEBFULLNAME="Griswald Brooks"
```
Archive the source code for use with `dh_make`
```
cd rosless/
tar -czf say-hello_1.0.orig.tar.gz say-hello/
```
Generate the package structure and choose `library` as the package type
```
cd say-hello/
dh_make -p say-hello_1.0
```
You should see
```
Type of package: (single, indep, library, python)
[s/i/l/p]?
Maintainer Name     : Griswald Brooks
Email-Address       : griswald.brooks@gmail.com
Date                : Mon, 18 Jul 2022 19:41:47 -0700
Package Name        : say-hello
Version             : 1.0
License             : blank
Package Type        : library
Are the details correct? [Y/n/q]
Skipping creating ../say-hello_1.0.orig.tar.gz because it already exists
Currently there is not top level Makefile. This may require additional tuning
Done. Please edit the files in the debian/ subdirectory now.


Make sure you edit debian/control and change the Package: lines from
say-helloBROKEN to something else, such as say-hello1
```
Update the `control` file
```
vim debian/control
```
Add `cmake` to `Build-Depends`, remove `BROKEN` sections, update `Depends` section.
The result should look like
```
Source: say-hello
Priority: optional
Maintainer: Griswald Brooks <griswald.brooks@gmail.com>
Build-Depends: debhelper-compat (= 12), cmake
Standards-Version: 4.4.1
Section: libs
Homepage: <insert the upstream URL, if relevant>
#Vcs-Browser: https://salsa.debian.org/debian/say-hello
#Vcs-Git: https://salsa.debian.org/debian/say-hello.git

Package: say-hello-dev
Section: libdevel
Architecture: any
Multi-Arch: same
Depends: ${misc:Depends}
Description: Decorates a string
```
Update the `copyright` file
```
vim debian/copyright
```
to be
```
Format: https://www.debian.org/doc/packaging-manuals/copyright-format/1.0/

Files: *
Copyright: 2022, Griswald Brooks
License: Expat

License: Expat
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 .
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 .
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
```

Remove extra `.install` files since there is only one library
```
rm debian/say-hello1.install debian/say-hello-dev.install
```
Create the debian. This will build the code.
From within the `say-hello` directory run
```
dpkg-buildpackage
```
In the parent directory, you will now see the debian
```
$ ls ..
compose.dev.yml  LICENCE    say-hello                        say-hello_1.0-1_amd64.changes  say-hello_1.0-1.dsc        say-hello-dev_1.0-1_amd64.deb
Dockerfile       README.md  say-hello_1.0-1_amd64.buildinfo  say-hello_1.0-1.debian.tar.xz  say-hello_1.0.orig.tar.gz
```
From the `say-hello` directory, clean the build artifacts
```
debclean
```

## PPA
Once the debian is created, it has to be hosted somewhere accessible to the target devices.
This location is referred to as a Personal Package Archive (PPA). The directory structure of a PPA looks like
```
└── ppa
    ├── InRelease
    ├── KEY.gpg
    ├── Packages
    ├── Packages.gz
    ├── Release
    ├── Release.gpg
    ├── package-name.deb
    └── package-sources.list
```
The parts here are
- Signed release files
  - `KEY.gpg`, public key
  - `Release, Release.gpg, InRelease`, signed files
-Package meta data
  - `Packages, Packages.gz`
- Debian binary
  - `package-name.deb`
  - This is also often named `package-name_version_arch.deb`
  - A PPA can container multiple debians or debian versions
- Sources list
  - `package-sources.list`
  - This tells `apt` where to look for packages
  - This file is installed locally as `/etc/apt/sources.list.d/package-sources.list`
  - The name of this file locally or in the ppa is not relevant, but the extension must be `.list`

### Updating the PPA
Move debian files into package directory
```
mv rosless/say-hello_1.0* ubuntu/
mv rosless/say-hello-dev_1.0-1_amd64.deb ubuntu/
```
While the `.deb` is all that is required, the other files allow debians to be
reproduced.
To sign the package, first create a key pair
```
gpg --full-gen-key
```
This is an interactive prompt.
Choose the options

- `(1) RSA and RSA (default)`
- Keysize `4096`
- Key is valid for `0 = key does not expire`
- Enter your name and email address

Export your public key to the ppa
```
gpg --armor --export $DEBEMAIL > ppa/KEY.gpg
```
Create the package meta data
> :warning: you must `cd` into the directory or the created debian will
refer to a duplicate directory that doesn't exist.

```
cd ubuntu/
dpkg-scanpackages --multiversion . > Packages
gzip -k -f Packages
```
Sign the package
```
apt-ftparchive release . > Release
gpg --default-key $DEBEMAIL -abs -o - Release > Release.gpg
gpg --default-key $DEBEMAIL --clearsign -o - Release > InRelease
```
Create the sources list
```
echo "deb http://griswaldbrooks.com/ppa/ubuntu/ ./" > package-sources.list
```
> :warning: The address can be `http` instead of `https` but cannot redirect.

The package should now be complete.
Push the updated repo.

### Adding repository to docker image
Now that the ppa is available, the target such as the docker image, needs to add it to `apt`
```
sudo curl -s --compressed "http://griswaldbrooks.com/ppa/ubuntu/KEY.gpg" | sudo apt-key add -
sudo curl -s --compressed -o /etc/apt/sources.list.d/say-hello.list "http://griswaldbrooks.com/ppa/ubuntu/say-hello-sources.list"
sudo apt update
```

Without using `rosdep`, the package could now be installed using
```
sudo apt install say-hello-dev
```
Which could be useful for testing.
To list the installed files
```
dpkg -L say-hello-dev
```
To remove the package
```
dpkg -P say-hello-dev
```
## rosdep
`/etc/ros/rosdep/sources.list.d` directory contains a list of sources that rosdep will use to find packages.
While in this example, the package manager used is `apt`, rosdep allows the use of other managers, such as `pip`.

Sources are described in `yaml` files which map rosdep package names to OS, OS version, and package manager. Using the defaults and assuming no difference in OS version, the `yaml` can be as simple as
```
say-hello-dev:
        ubuntu: [say-hello-dev]
```
For the general case, this would be put in a file called `base.yaml` and hosted in an accessible location such as
```
http://griswaldbrooks.com/ppa/rosdistro/base.yaml
```
The source for this can be added to the container
```
sudo sh -c 'echo "yaml http://griswaldbrooks.com/ppa/rosdistro/base.yaml" > /etc/ros/rosdep/sources.list.d/1-private.list'
```
Checking the `sources.list.d`
```
$ ls /etc/ros/rosdep/sources.list.d/
1-private.list  20-default.list
```
> :warning: The new list does not have to be named *-private.list but should be prefixed with a number smaller than 20 as rosdep will check the file with the smallest number first and uses the first source for a package first. rosdep does not merge source entries, even if there is no conflict.

## References

- [rosdep 0.22.1 documentation](https://docs.ros.org/en/independent/api/rosdep/html/overview.html)
- [Does rosdep support installing dependencies from PPAs?](https://answers.ros.org/question/70210/does-rosdep-support-installing-dependencies-from-ppas/)
- [Linux dpkg command cheat sheet](https://www.cyberciti.biz/howto/question/linux/dpkg-cheat-sheet.php)
- [Hosting your own PPA repository on GitHub](https://assafmo.github.io/2019/05/02/ppa-repo-hosted-on-github.html)
- [Easy way to create a Debian package and local package repository](https://linuxconfig.org/easy-way-to-create-a-debian-package-and-local-package-repository)
- [How to create a .deb file](https://askubuntu.com/a/493577)

## Appendix
- While not best practice, legacy debian build instructions using `dpkg-deb` are [here](docs/LEGACY-debian-build.md)
