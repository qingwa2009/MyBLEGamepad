set(component_srcs)

idf_component_register(SRCS "MyMeanFilter.c" "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES ""
                       REQUIRES "")
