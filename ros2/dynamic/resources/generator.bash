package_name=""
dependencies=""
install_path=""

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -p|--package_name) package_name="$2"; shift ;;
        -i|--install_path) install_path="$2"; shift ;;
        *)
          if [[ $1 == *"--"* ]]; then
                param="${1/--/}"
                declare $param="$2"
                echo $1 $2
          fi ;;
    esac
    shift
done

for pkg in $package_name; do

  echo "Generating Type Support for package $pkg with dependencies ${!pkg}";

  cp /tmp/is_ros2_sh/CMakeLists.txt /tmp/$pkg/

  sed "s#<name>\([^<][^<]*\)</name>#<name>$pkg</name>#" /tmp/is_ros2_sh/package.xml > /tmp/$pkg/package.xml

  cd /tmp/$pkg

  echo "Installing the generated Type Support in $install_path";

  echo "Command: cmake . -DIS_PACKAGE_NAME=$pkg -DPACKAGE_DEPENDENCIES=${!pkg} -DCMAKE_INSTALL_PREFIX:PATH=$install_path";

  cmake . -DIS_PACKAGE_NAME=$pkg -DPACKAGE_DEPENDENCIES=${!pkg} -DCMAKE_INSTALL_PREFIX:PATH=$install_path

  cmake --build . --target install

  if [ $? -ne 0 ]
  then
    exit 1
  fi

done

. $install_path/setup.bash
