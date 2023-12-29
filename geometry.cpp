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
using point_t = boost::geometry::model::d2::point_xy<double>;
using PointObject = struct {
    PyObject_HEAD
    point_t* point;
};

// point new function
extern "C" PyObject* point_new(PyTypeObject* type, PyObject*, PyObject*) {
    PointObject *self = (PointObject*)type->tp_alloc(type, 0);
    self->point = new(point_t);
    return (PyObject*)self;
}

// point deallocation function
extern "C" void point_dealloc(PointObject* self) {
    delete(self->point);
    Py_TYPE(self)->tp_free((PyObject*)self);
}

// point init function
extern "C" int point_init(PointObject* self, PyObject* args, PyObject*) {
    double x, y;
    if (!PyArg_ParseTuple(args, "dd", &x, &y)) return -1;
    self->point->x(x);
    self->point->y(y);
    return 0;
}

// point x method returns the value of x.
extern "C" PyObject* point_x(PointObject* self, PyObject*) {
    return PyFloat_FromDouble(self->point->x());
}

// point y method returns the value of y.
extern "C" PyObject* point_y(PointObject* self, PyObject*) {
    return PyFloat_FromDouble(self->point->y());
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
    .tp_dealloc =(destructor)point_dealloc,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR("point objects"),
    .tp_methods = point_methods,
    .tp_init = (initproc)point_init,
    .tp_new = point_new
};

// polygon object definition
using polygon_t = boost::geometry::model::polygon<point_t>;
using PolygonObject = struct {
    PyObject_HEAD
    polygon_t* polygon;
};

// polygon new function
extern "C" PyObject* polygon_new(PyTypeObject* type, PyObject*, PyObject*) {
    PolygonObject *self = (PolygonObject*)type->tp_alloc(type, 0);
    self->polygon = new(polygon_t);
    return (PyObject*)self;
}

// polygon deallocation function
extern "C" void polygon_dealloc(PolygonObject* self) {
    delete(self->polygon);
    Py_TYPE(self)->tp_free((PyObject*)self);
}

// polygon append method
extern "C" PyObject* polygon_append(PolygonObject* self, PyObject* p) {
    if (PyObject_TypeCheck(p, &PointType) ) {
        boost::geometry::append(*(self->polygon), *((PointObject*)p)->point);
    } else {
        PyErr_SetString(PyExc_TypeError, "only points can be appended to polygons");
        return nullptr;
    }
    Py_INCREF(self);
    return (PyObject*)self;
}

// polygon x method returns the x values.
extern "C" PyObject* polygon_x(PolygonObject* self, PyObject*) {
    auto x = PyList_New(0);
    for (auto& p: self->polygon->outer() ) {
        PyList_Append(x, PyFloat_FromDouble(p.x()));
    }
    return x;
}

// polygon y method returns the y values.
extern "C" PyObject* polygon_y(PolygonObject* self, PyObject*) {
    auto y = PyList_New(0);
    for (auto& p: self->polygon->outer() ) {
        PyList_Append(y, PyFloat_FromDouble(p.y()));
    }
    return y;
}

// polygon methods
static PyMethodDef polygon_methods[] = {
    {"append", (PyCFunction)polygon_append, METH_O, "Append point"},
    {"x", (PyCFunction)polygon_x, METH_NOARGS, "Return list of x coordinates"},
    {"y", (PyCFunction)polygon_y, METH_NOARGS, "Return list of y coordinates"},
    {nullptr}
};

// polygon type definition
static PyTypeObject PolygonType = {
    .ob_base = PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "geometry.polygon",
    .tp_basicsize = sizeof(PolygonObject),
    .tp_itemsize = 0,
    .tp_dealloc =(destructor)polygon_dealloc,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR("polygon objects"),
    .tp_methods = polygon_methods,
    .tp_new = polygon_new
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
    if (PyType_Ready(&PointType) < 0) return nullptr;
    if (PyType_Ready(&PolygonType) < 0) return nullptr;

    // Create module
    PyObject* m = PyModule_Create(&geometrymodule);
    if (m == NULL)
        return NULL;

    // Add types
    Py_INCREF(&PointType);
    if (PyModule_AddObject(m, "point", (PyObject*) &PointType) < 0) {
        Py_DECREF(&PointType);
        Py_DECREF(m);
        return nullptr;
    }
    Py_INCREF(&PolygonType);
    if (PyModule_AddObject(m, "polygon", (PyObject*) &PolygonType) < 0) {
        Py_DECREF(&PolygonType);
        Py_DECREF(m);
        return nullptr;
    }
    return m;
}
