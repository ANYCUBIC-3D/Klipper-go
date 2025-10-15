package jinja2

import (
	pongo2 "github.com/flosch/pongo2/v5"
)

type Environment struct {
	templateSet *pongo2.TemplateSet
}

type Template struct {
	engine *pongo2.Template
}

func NewEnvironment() *Environment {
	return &Environment{templateSet: pongo2.NewSet("default", pongo2.DefaultLoader)}
}

func (r *Environment) From_string(tpl string) (*Template, error) {
	t, err := r.templateSet.FromString(tpl)

	if err != nil {
		return nil, err
	}
	return &Template{engine: t}, nil
}

func (t *Template) Render(ctx pongo2.Context) (string, error) {
	return t.engine.Execute(ctx)
}
