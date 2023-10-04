# CognitivePilot
Odometry coefficients estimation using Kalman Filter

Чтобы воспользоваться проектом, может потребоваться Visual Studio.
Один из вариантов - "Файл" -> "Создать" -> "Создать проект из существующего кода" и выбрать папку с расположенными там файлами кода.
Далее необходимо в "Типе проекта" указать "Проект консольное приложение". После достаточно нажать "Готово".

## Как собрать

Создаем директорию для сборки:

```
mkdir build
cd build
```

Устанавливаем зависимости через конан:

```
conan install .. --build=missing
```

Проектная генерация cmake:

```
cmake .. -GNinja --profile=release 
```

Сборка

```
cd Release
ninja
```

## Запуск

Рабочая директория запуска должна содержать входной файл `mxm_data.txt`

```
cd ..
build/mylittlekalman
```

после успешного выполнения будет выведено 

```
DataReading:
Press 'Enter' to quit
```

а в директории появятся два файла `output.txt` и `covariance.txt`

