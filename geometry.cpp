/**
 * Python interface to a tiny subset of boost::geometry
*/
#define PY_SSIZE_T_CLEAN
#include <Python.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#pragma GCC diagnostic pop

// point object definition
using PointObject = struct {
    PyObject_HEAD
    boost::geometry::model::d2::point_xy<double> point;
};

//point init function
extern "C" int point_init(PointObject* self, PyObject* args, PyObject*) {
    double x, y;
    if (!PyArg_ParseTuple(args, "dd", &x, &y)) return -1;
    self->point.x(x);
    self->point.y(y);
    return 0;
}

// point x method returns the value of x.
extern "C" PyObject* point_x(PointObject* self, PyObject*) {
    return PyFloat_FromDouble(self->point.x());
}

// point y method returns the value of y.
extern "C" PyObject* point_y(PointObject* self, PyObject*) {
    return PyFloat_FromDouble(self->point.y());
}

// point methods
static PyMethodDef point_methods[] = {
    {"x", (PyCFunction)point_x, METH_NOARGS, "Return x"},
    {"y", (PyCFunction)point_y, METH_NOARGS, "Return y"},
    {nullptr}
};

// point type definition
static PyTypeObject PointType = {
    .ob_base = PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "geometry.point",
    .tp_basicsize = sizeof(PointObject),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR("point objects"),
    .tp_methods = point_methods,
    .tp_init = (initproc)point_init,
    .tp_new = PyType_GenericNew
};

// geometry module definition
static PyModuleDef geometrymodule = {
    .m_base = PyModuleDef_HEAD_INIT,
    .m_name = "geometry",
    .m_doc = "minimal interface to boost geometry.",
    .m_size = -1,
};

// geometry module init function
PyMODINIT_FUNC PyInit_geometry(void)
{
    // Define types
    PyObject *m;
    if (PyType_Ready(&PointType) < 0)
        return NULL;

    // Create module
    m = PyModule_Create(&geometrymodule);
    if (m == NULL)
        return NULL;

    // Add types
    Py_INCREF(&PointType);
    if (PyModule_AddObject(m, "point", (PyObject*) &PointType) < 0) {
        Py_DECREF(&PointType);
        Py_DECREF(m);
        return nullptr;
    }
    return m;
}
