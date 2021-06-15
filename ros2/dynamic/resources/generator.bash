package_name=""
dependencies=""
install_path=""

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -p|--package_name) package_name="$2"; shift ;;
        -d|--dependencies) dependencies="$2"; shift ;;
        -i|--install_path) install_path="$2"; shift ;;
        *) echo "Unknown parameter passed: $1" ;;
    esac
    shift
done

echo "Generating Type Support for package $package_name with dependencies $dependencies";

cp /tmp/CMakeLists.txt /tmp/$package_name/

sed "s#<name>\([^<][^<]*\)</name>#<name>$package_name</name>#" /tmp/package.xml > /tmp/$package_name/package.xml

cd /tmp/$package_name

echo "Installing the generated Type Support in $install_path";

cmake . -DIS_PACKAGE_NAME=$package_name -DPACKAGE_DEPENDENCIES=$dependencies -DCMAKE_INSTALL_PREFIX:PATH=$install_path

cmake --build . --target install

if [ $? -ne 0 ]
then
  exit 1
fi

. $install_path/setup.bash