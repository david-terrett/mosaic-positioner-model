/**
 * Python interface to a tiny subset of boost::geometry
*/
#define PY_SSIZE_T_CLEAN
#include "Python.h"

#include <iomanip>
#include <sstream>

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

// point __repr__ method
extern "C" PyObject* point_repr(PointObject* self, PyObject*) {
    std::ostringstream repr;
    repr << std::setprecision(16);
    repr << "point(" << self->point->x() << "," << self->point->y() << ")";
    return (PyObject*)PyUnicode_DecodeUTF8(repr.str().c_str(),
            repr.str().size(), nullptr);
}

// point methods
static PyMethodDef point_methods[] = {
    {"x", (PyCFunction)point_x, METH_NOARGS, "Return x"},
    {"y", (PyCFunction)point_y, METH_NOARGS, "Return y"},
    {nullptr, nullptr, 0, nullptr}
};

// point type definition
static PyTypeObject PointType = {
    .ob_base = PyVarObject_HEAD_INIT(nullptr, 0)
    .tp_name = "geometry.point",
    .tp_basicsize = sizeof(PointObject),
    .tp_itemsize = 0,
    .tp_dealloc =(destructor)point_dealloc,
    .tp_vectorcall_offset = 0,
    .tp_getattr = (getattrfunc)nullptr,
    .tp_setattr = (setattrfunc)nullptr,
    .tp_as_async = (PyAsyncMethods*)nullptr,
    .tp_repr = (reprfunc)point_repr,
    .tp_as_number = (PyNumberMethods*)nullptr,
    .tp_as_sequence = (PySequenceMethods*)nullptr,
    .tp_as_mapping = (PyMappingMethods*)nullptr,
    .tp_hash = (hashfunc)nullptr,
    .tp_call = (ternaryfunc)nullptr,
    .tp_str = (reprfunc)nullptr,
    .tp_getattro = (getattrofunc)nullptr,
    .tp_setattro = (setattrofunc)nullptr,
    .tp_as_buffer = (PyBufferProcs*)nullptr,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR("point objects"),
    .tp_traverse = (traverseproc)nullptr,
    .tp_clear = (inquiry)nullptr,
    .tp_richcompare = (richcmpfunc)nullptr,
    .tp_weaklistoffset = 0,
    .tp_iter = (getiterfunc)nullptr,
    .tp_iternext = (iternextfunc)nullptr,
    .tp_methods = point_methods,
    .tp_members = (struct PyMemberDef*)nullptr,
    .tp_getset = (struct PyGetSetDef*)nullptr,
    .tp_base = (PyTypeObject*)nullptr,
    .tp_dict = (PyObject*)nullptr,
    .tp_descr_get = (descrgetfunc)nullptr,
    .tp_descr_set = (descrsetfunc)nullptr,
    .tp_dictoffset = 0,
    .tp_init = (initproc)point_init,
    .tp_alloc = (allocfunc)nullptr,
    .tp_new = point_new,
    .tp_free = (freefunc)nullptr,
    .tp_is_gc = (inquiry)nullptr,
    .tp_bases = (PyObject*)nullptr,
    .tp_mro = (PyObject*)nullptr,
    .tp_cache = (PyObject*)nullptr,
    .tp_subclasses = nullptr,
    .tp_weaklist = (PyObject*)nullptr,
    .tp_del = (destructor)nullptr,
    .tp_version_tag = 0,
    .tp_finalize = (destructor)nullptr,
    .tp_vectorcall = (vectorcallfunc)nullptr
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

// polygon points method returns the y values.
extern "C" PyObject* polygon_points(PolygonObject* self, PyObject*) {
    auto points = PyList_New(0);
    for (auto& p: self->polygon->outer() ) {
        PointObject* p1 = PyObject_New(PointObject, &PointType);
        p1->point = new(point_t);
        p1->point->x(p.x());
        p1->point->y(p.y());
        PyList_Append(points, (PyObject*)p1);
    }
    return points;
}

// polygon methods
static PyMethodDef polygon_methods[] = {
    {"append", (PyCFunction)polygon_append, METH_O, "Append point"},
    {"x", (PyCFunction)polygon_x, METH_NOARGS, "Return list of x coordinates"},
    {"y", (PyCFunction)polygon_y, METH_NOARGS, "Return list of y coordinates"},
    {"points", (PyCFunction)polygon_points, METH_NOARGS, "Return list of points"},
    {nullptr, nullptr, 0, nullptr}
};

// polygon type definition
static PyTypeObject PolygonType = {
    .ob_base = PyVarObject_HEAD_INIT(nullptr, 0)
    .tp_name = "geometry.polygon",
    .tp_basicsize = sizeof(PolygonObject),
    .tp_itemsize = 0,
    .tp_dealloc =(destructor)polygon_dealloc,
    .tp_vectorcall_offset = 0,
    .tp_getattr = (getattrfunc)nullptr,
    .tp_setattr = (setattrfunc)nullptr,
    .tp_as_async = (PyAsyncMethods*)nullptr,
    .tp_repr = (reprfunc)nullptr,
    .tp_as_number = (PyNumberMethods*)nullptr,
    .tp_as_sequence = (PySequenceMethods*)nullptr,
    .tp_as_mapping = (PyMappingMethods*)nullptr,
    .tp_hash = (hashfunc)nullptr,
    .tp_call = (ternaryfunc)nullptr,
    .tp_str = (reprfunc)nullptr,
    .tp_getattro = (getattrofunc)nullptr,
    .tp_setattro = (setattrofunc)nullptr,
    .tp_as_buffer = (PyBufferProcs*)nullptr,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR("polygon objects"),
    .tp_traverse = (traverseproc)nullptr,
    .tp_clear = (inquiry)nullptr,
    .tp_richcompare = (richcmpfunc)nullptr,
    .tp_weaklistoffset = 0,
    .tp_iter = (getiterfunc)nullptr,
    .tp_iternext = (iternextfunc)nullptr,
    .tp_methods = polygon_methods,
    .tp_members = (struct PyMemberDef*)nullptr,
    .tp_getset = (struct PyGetSetDef*)nullptr,
    .tp_base = (PyTypeObject*)nullptr,
    .tp_dict = (PyObject*)nullptr,
    .tp_descr_get = (descrgetfunc)nullptr,
    .tp_descr_set = (descrsetfunc)nullptr,
    .tp_dictoffset = 0,
    .tp_init = (initproc)nullptr,
    .tp_alloc = (allocfunc)nullptr,
    .tp_new = polygon_new,
    .tp_free = (freefunc)nullptr,
    .tp_is_gc = (inquiry)nullptr,
    .tp_bases = (PyObject*)nullptr,
    .tp_mro = (PyObject*)nullptr,
    .tp_cache = (PyObject*)nullptr,
    .tp_subclasses = nullptr,
    .tp_weaklist = (PyObject*)nullptr,
    .tp_del = (destructor)nullptr,
    .tp_version_tag = 0,
    .tp_finalize = (destructor)nullptr,
    .tp_vectorcall = (vectorcallfunc)nullptr
};

// distance function
extern "C" PyObject* distance(PyObject*, PyObject* args) {
    PointObject *p1, *p2;
    if (!PyArg_ParseTuple(args, "OO", &p1, &p2)) return nullptr;
    double result(0.0);
    if (PyObject_TypeCheck(p1, &PointType) &&
            PyObject_TypeCheck(p2, &PointType)) {
        result = boost::geometry::distance(*p1->point, *p2->point);
    } else {
        PyErr_SetString(PyExc_TypeError, "distance only supports points");
        return nullptr;
    }
    return PyFloat_FromDouble(result);
};


// Intersects function
extern "C" PyObject* intersects(PyObject*, PyObject* args) {
    PolygonObject *p1, *p2;
    if (!PyArg_ParseTuple(args, "OO", &p1, &p2)) return nullptr;
    bool result(false);
    if (PyObject_TypeCheck(p1, &PolygonType) &&
            PyObject_TypeCheck(p2, &PolygonType)) {
        result = boost::geometry::intersects(*p1->polygon, *p2->polygon);
    } else {
        PyErr_SetString(PyExc_TypeError, "only polygons can intersect");
        return nullptr;
    }
    return PyBool_FromLong((long)result);
};

// geometry module functions
static PyMethodDef geometry_methods[] = {
    {"distance", (PyCFunction)distance, METH_VARARGS, "distance between points"},
    {"intersects", (PyCFunction)intersects, METH_VARARGS, "polygons intersect"},
    {nullptr, nullptr, 0, nullptr}
};

// geometry module definition
static PyModuleDef geometrymodule = {
    .m_base = PyModuleDef_HEAD_INIT,
    .m_name = "geometry",
    .m_doc = "minimal interface to boost geometry.",
    .m_size = -1,
    .m_methods = geometry_methods,
    .m_slots = (PyModuleDef_Slot*)0,
    .m_traverse = (traverseproc)0,
    .m_clear = (inquiry)0,
    .m_free = (freefunc)0
};

// geometry module init function
PyMODINIT_FUNC PyInit_geometry(void)
{
    // Define types
    if (PyType_Ready(&PointType) < 0) return nullptr;
    if (PyType_Ready(&PolygonType) < 0) return nullptr;

    // Create module
    PyObject* m = PyModule_Create(&geometrymodule);
    if (m == nullptr)
        return nullptr;

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
