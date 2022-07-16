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
sudo apt install debian-package-name
```

This differs from a debian source package which also contains source code and instructions for building the binary. Creating debian source packages is not required to create debian binaries.

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
For packages to be used in other C++ projects, a debian might install to

```
/usr/include/package-name/header-name.h
/usr/lib/x86_64-linux-gnu/libpackage-name.so
```

### Creating a debian
To create a debian binary a directory structure containing the target install locations and metadata control file is made. For example, the below procedure for packaging a single binary will produce
```
$ tree package-name
package-name
|-- DEBIAN
|   `-- control
`-- usr
    `-- bin
        `-- package-executable
```
Creating the package is then
```
mkdir -p package-name/DEBIAN
touch package-name/DEBIAN/control
mkdir -p package-name/usr/bin/
cp /path/to/package/package-executable package-name/usr/bin/
```
Update the `package-name/DEBIAN/control` file
```
Package: say-hello
Version: 1.0
Section: custom
Priority: optional
Architecture: all
Essential: no
Installed-Size: 1024
Maintainer: Griswald Brooks <griswald.brooks@gmail.com>
Description: Decorates a string
```
Building the package is then
```
dpkg-deb --build package-name
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
  - This tells apt where to look for packages
  - This file is installed locally as `/etc/apt/sources.list.d/package-sources.list`
  - The name of this file locally or in the ppa is not relevant, but the extension must be `.list`

### Updating the PPA
Create your local package with debian
```
mkdir ppa
cp /path/to/package-name.deb ppa/
```
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
gpg --armor --export "${email address}" > ppa/KEY.gpg
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
gpg --default-key "${email address}" -abs -o - Release > Release.gpg
gpg --default-key "${email address}" --clearsign -o - Release > InRelease
```
Create the sources list
```
echo "deb https://url.private.server/ppa ./" > ppa/package-sources.list
```
The package should now be complete.
Upload the `ppa/` directory to `https://url.private.server/ppa`

> :warning: The address can be http instead of https but cannot redirect.

### Adding repository to docker image
Now that the ppa is available, the target such as the docker image, needs to add it to `apt`
```
curl -s --compressed "https://url.private.server/ppa/KEY.gpg" | sudo apt-key add -
sudo curl -s --compressed -o /etc/apt/sources.list.d/ottava.list "https://url.private.server/ppa/package-sources.list"
sudo apt update
```

Without using `rosdep`, the package could now be installed using
```
sudo apt install package-name
```
Which could be useful for testing.
To list the installed files
```
dpkg -L package-name
```
To remove the package
```
dpkg -P package-name
```
## rosdep
`/etc/ros/rosdep/sources.list.d` directory contains a list of sources that rosdep will use to find packages.
While in this example, the package manager used is `apt`, rosdep allows the use of other managers, such as `pip`.

Sources are described in `yaml` files which map rosdep package names to OS, OS version, and package manager. Using the defaults and assuming no difference in OS version, the `yaml` can be as simple as
```
package-name:
  ubuntu: [package-name]
```
For the general case, this would be put in a file called `base.yaml` and hosted in an accessible location such as
```
https://url.private.server/base.yaml
```
The source for this can be added to the container
```
sudo sh -c 'echo "yaml https://url.private.server/base.yaml" > /etc/ros/rosdep/sources.list.d/1-private.list'
```
Checking the `sources.list.d`
```
$ ls /etc/ros/rosdep/sources.list.d/
1-private.list  20-default.list
```
> :warning: The new list does not have to be named *-private.list but should be prefixed with a number larger than 20 as rosdep will check the file with the smallest number first and uses the first source for a package first. rosdep does not merge source entries, even if there is no conflict.

## References

- [rosdep 0.22.1 documentation](https://docs.ros.org/en/independent/api/rosdep/html/overview.html)
- [Does rosdep support installing dependencies from PPAs?](https://answers.ros.org/question/70210/does-rosdep-support-installing-dependencies-from-ppas/)
- [Linux dpkg command cheat sheet](https://www.cyberciti.biz/howto/question/linux/dpkg-cheat-sheet.php)
- [Hosting your own PPA repository on GitHub](https://assafmo.github.io/2019/05/02/ppa-repo-hosted-on-github.html)
- [Easy way to create a Debian package and local package repository](https://linuxconfig.org/easy-way-to-create-a-debian-package-and-local-package-repository)
- [How to create a .deb file](https://askubuntu.com/a/493577)
