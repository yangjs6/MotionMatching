#include "array.h"

// template <typename T>
// void array1d<T>::resize(int _size)
// {
//     if (_size == 0 && size != 0)
//     {
//         free(data);
//         data = NULL;
//         size = 0;
//     }
//     else if (_size > 0 && size == 0)
//     {
//         data = (T*)malloc(_size * sizeof(T));
//         size = _size;
//         assert(data != NULL);
//     }
//     else if (_size > 0 && size > 0 && _size != size)
//     {
//         data = (T*)realloc(data, _size * sizeof(T));
//         size = _size;
//         assert(data != NULL);           
//     }
// }

// template <typename T>
// void array2d<T>::resize(int _rows, int _cols)
// {
//     int _size = _rows * _cols;
//     int size = rows * cols;
//         
//     if (_size == 0 && size != 0)
//     {
//         free(data);
//         data = NULL;
//         rows = 0;
//         cols = 0;
//     }
//     else if (_size > 0 && size == 0)
//     {
//         data = (T*)malloc(_size * sizeof(T));
//         rows = _rows;
//         cols = _cols;
//         assert(data != NULL);
//     }
//     else if (_size > 0 && size > 0 && _size != size)
//     {
//         data = (T*)realloc(data, _size * sizeof(T));
//         rows = _rows;
//         cols = _cols;
//         assert(data != NULL);           
//     }
// }