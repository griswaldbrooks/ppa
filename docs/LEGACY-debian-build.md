# DEPRECIATED
While these instructions work to create a debian package that can be installed,
they omit a number of files required for best practices.

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
