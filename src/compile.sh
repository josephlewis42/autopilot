cd `dirname $0`
mkdir "compile"

CPPS=`find . -iname "*.cc" -o -iname "*.cpp" | grep -v tests`
HS=`find . -iname "*.h" -o -iname "*.hpp" | grep -v tests`

cp $CPPS "compile"
cp $HS "compile"

cd "compile"
for file in *.cpp; do
    mv "$file" "`basename $file .cpp`.cc"
done
