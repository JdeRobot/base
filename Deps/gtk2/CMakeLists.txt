include(FindPkgConfig)

PKG_CHECK_MODULES(gtk20 REQUIRED gtk+-2.0)
PKG_CHECK_MODULES(gtkgl20 REQUIRED gtkgl-2.0)
PKG_CHECK_MODULES(gtkmm REQUIRED gtkmm-2.4)
PKG_CHECK_MODULES(libglademm REQUIRED libglademm-2.4)
PKG_CHECK_MODULES(gthread REQUIRED gthread-2.0)
PKG_CHECK_MODULES(libgnomecanvas REQUIRED libgnomecanvas-2.0)
PKG_CHECK_MODULES(libgnomecanvasmm REQUIRED libgnomecanvasmm-2.6)
PKG_CHECK_MODULES(gtkglextmm REQUIRED gtkglextmm-1.2)


SET(DEPS "${DEPS} libgtk2.0-0, libgtkgl2.0-1, libgtkmm-2.4-1c2a, libgtkglext1, libglademm-2.4-1c2a, libgnomecanvas2-0, libgnomecanvasmm-2.6-1c2a, libgnomecanvasmm-2.6-1c2a, libgtkglextmm-x11-1.2-0, libgtkglext1,")
SET(DEPS_DEV "${DEPS_DEV} libgtk2.0-bin, libgtk2.0-cil, libgtk2.0-common, libgtk2.0-dev, libgtkgl2.0-dev, libgtkglext1-doc, libgtkglext1-dev, libgtkmm-2.4-dev, libglademm-2.4-dev, libgnomecanvas2-dev, libgnomecanvasmm-2.6-dev, libgtkglextmm-x11-1.2-dev,")
