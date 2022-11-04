## Instrukcja obsługi narzędzi do skanowania obiektów przy użyciu kilku Lidarów L515

Programy umożliwią skanowanie obiektów lub odczyt wcześniej zeskanowanych plików.

W celu włączenia skanowania używamy opcji: --lidar

W celu wczytania wcześniej zeskanowanych plików używamy opcji: --input_folder=<katalog gdzie znajdują się pliki z rozszerzeniem ply>. Np. --input_folder="Kasia/1"

Skanowanie obiektów odbywa się zgodnie z opisem https://dev.intelrealsense.com/docs/lidar-camera-l515-multi-camera-setup. Sterowanie poszczególnymi kamerami odbywa się za pomocą układu FT232H podłączonego do tej samej stacji co L515.

Dostępne są dwa narzędzia:
1. Główne narzędzie skanujące - main.py
2. Narzędzie pomocnicze do automatycznego wyliczenia izomorficznych transformacji przestrzennych poszczególnych point cloud względem układu współrzędnych kamery - auto_transform_calculator.py

###### Podłączenie lidarów z portami GPIO FT232H
Konfiguracja FT232H: https://learn.adafruit.com/circuitpython-on-any-computer-with-ft232h/linux

Oba narzędzia wymagają wprowadzenia jako zmiennej środowiskowej wartości: BLINKA_FT232H=1
W pliku `scene_reader.py` znajduje się definicja dictionary skałdająca się z trzech ostatnich cyfr numeru seryjnego L515 oraz powiązanego z nim porty GPIO:

`        self.gpio_config = {
            '303': board.C1,
            '608': board.C3,
            '337': board.C2,
            '630': board.C0
        }`

  W pyrzpadku innego podłączenia należy zmienić powyższą konfigurację.


### Narzędzie do automatycznego wyliczania transformacji

1. Uruchomienie narzędzia: `python3 auto_transform_calculator.py`
2. Narzędzie automatycznie wykrywa ilość podłączonych lidarów.
3. Po odczytaniu danych z lidarów okna otwierają się parami: SOURCE, TARGET. Narzędzie będzie szukać transformacji SOURCE, żeby pasował do TARGET. Pierwszy podłączony lidar staję się pierwszym TARGET.
4. Na point cloud TARGET i SOURCE zaznaczamy punkty korespondencyjne. Minimum 3. Zaznaczenie/odznaczenie wykonujemy poprzez SHIFT + lewy/prawy przycisk myszy.
5. Jeżeli jesteśmy pewni zaznaczenia to zamykamy okna klawiszem `q`.
6. Pojawia sie okno poglądowe jak wygląda SOURCE w stosunku do TARGET po dokonaniu transformacji. Zamykamy okno przez `q`.
7. Jeżeli jest więcej lidarów, to pojawi się kolejna para okien TARGET i SOURCE, ale tym razem TARGET będzie zawierał dwa poprzednie point cloud po dokonaniu ostatniej transformacji.
8. Procedurę kontynuujemy do momentu przetworzenia wszystkich point cloud z wszystkich lidarów.
9. Jako wynik program generuje plik o nazwie `auto_transform_matrices.npy`, który jest automatycznie wczytywany przez główne narzędzie do skanowania.

### Narzędzie do automatycznego wyliczania transformacji

1. Uruchomienie narzędzia: `python3 main.py`
2. Narzędzie automatycznie wczytuje plik `auto_transform_matrices.npy`, który pozwala na wstępne przekształcenie danych z poszczególnych lidar'ów.
3. Następnie sekwencyjnie czytane są  poszczególne point clouds z lidar'ów.
4. Wszystkie odczytane point clouds wyświetlane są w jednym oknie. Transformacje na poszczególnych point clouds odbywają się poprzez wskazanie, które point could są aktywne za pomocą klawiszy: `F1, F2, F3, F4`. Na początku wszystkie odczytane point clouds są aktywne. Klawisz `F12` aktywuje wszystkie point cloud.
5. Lista transformacji aktywnych point clouds:
    * `A <-> D` przesunięcie lewo/prawo oś X
    * `W <-> S` przesunięcie lewo/prawo oś Y
    * `Z <-> X` przesunięcie lewo/prawo oś Z
    * `R <-> T` obrót lewo/prawo oś X
    * `F <-> G` obrót lewo/prawo oś Y
    * `V <-> B` obrót lewo/prawo oś Z
    * `Backspace` wykonanie transformacji wstecznej
    * `P` wyświetlenie macierzy transformacji izomorficznej
    * `O` zapisanie macierzy transformacji izomorficznej
    * `L` wczytanie macierzy transformacji izomorficznej
    * `I` eksport WSZYSTKICH point ploud w jednym pliku PLY
    * `.` voxel down sample
    * `,` radius filter
    * `M` utworzenie mesh'a
    * `/` zmiana wyświetlanych geometrii pount coulds / mesh
    * `U` eksport mesh'a do pliku OBJ
    * `E` reset pozycji kamery
    * `C` estymacja normals dla aktywncyh point clouds. UWAGA: pozycja kamery ma wpły w którą stronę będa skierowane wektory normals. Kierunek wektorów ma wpływ na proces tworzenia mesh'a.
6. Zamknięcie okna i zakończenie programu `q`.
7. Lista pozostałych domyślnych poleceń `h` 

    * [Open3D INFO]   -- Mouse view control --
    * [Open3D INFO]     Left button + drag         : Rotate.
    * [Open3D INFO]     Ctrl + left button + drag  : Translate.
    * [Open3D INFO]     Wheel button + drag        : Translate.
    * [Open3D INFO]     Shift + left button + drag : Roll.
    * [Open3D INFO]     Wheel                      : Zoom in/out.
 
    * [Open3D INFO]   -- Keyboard view control --
    * [Open3D INFO]     [/]          : Increase/decrease field of view.
    * [Open3D INFO]     R            : Reset view point.
    * [Open3D INFO]     Ctrl/Cmd + C : Copy current view status into the clipboard.
    * [Open3D INFO]     Ctrl/Cmd + V : Paste view status from clipboard.
 
    * [Open3D INFO]   -- General control --
    * [Open3D INFO]     Q, Esc       : Exit window.
    * [Open3D INFO]     H            : Print help message.
    * [Open3D INFO]     P, PrtScn    : Take a screen capture.
    * [Open3D INFO]     D            : Take a depth capture.
    * [Open3D INFO]     O            : Take a capture of current rendering settings.
    * [Open3D INFO]     Alt + Enter  : Toggle between full screen and windowed mode.
 
    * [Open3D INFO]   -- Render mode control --
    * [Open3D INFO]     L            : Turn on/off lighting.
    * [Open3D INFO]     +/-          : Increase/decrease point size.
    * [Open3D INFO]     Ctrl + +/-   : Increase/decrease width of geometry::LineSet.
    * [Open3D INFO]     N            : Turn on/off point cloud normal rendering.
    * [Open3D INFO]     S            : Toggle between mesh flat shading and smooth shading.
    * [Open3D INFO]     W            : Turn on/off mesh wireframe.
    * [Open3D INFO]     B            : Turn on/off back face rendering.
    * [Open3D INFO]     I            : Turn on/off image zoom in interpolation.
    * [Open3D INFO]     T            : Toggle among image render:
    * [Open3D INFO]                    no stretch / keep ratio / freely stretch.

    * [Open3D INFO]   -- Color control --
    * [Open3D INFO]     0..4,9       : Set point cloud color option.
    * [Open3D INFO]                    0 - Default behavior, render point color.,00
    * [Open3D INFO]                    1 - Render point color.
    * [Open3D INFO]                    2 - x coordinate as color.
    * [Open3D INFO]                    3 - y coordinate as color.
    * [Open3D INFO]                    4 - z coordinate as color.
    * [Open3D INFO]                    9 - normal as color.
    * [Open3D INFO]     Ctrl + 0..4,9: Set mesh color option.
    * [Open3D INFO]                    0 - Default behavior, render uniform gray color.
    * [Open3D INFO]                    1 - Render point color.
    * [Open3D INFO]                    2 - x coordinate as color.
    * [Open3D INFO]                    3 - y coordinate as color.
    * [Open3D INFO]                    4 - z coordinate as color.
    * [Open3D INFO]                    9 - normal as color.
    * [Open3D INFO]     Shift + 0..4 : Color map options.
    * [Open3D INFO]                    0 - Gray scale color.
    * [Open3D INFO]                    1 - JET color map.
    * [Open3D INFO]                    2 - SUMMER color map.
    * [Open3D INFO]                    3 - WINTER color map.
    * [Open3D INFO]                    4 - HOT color map.
     
    
.