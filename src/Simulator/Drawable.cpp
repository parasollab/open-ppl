#include "Drawable.h"

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Bodies/FixedBody.h"
#include "Geometry/Bodies/FreeBody.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Bodies/StaticMultiBody.h"


/*------------------------------- Construction -------------------------------*/

Drawable::
Drawable(MultiBody* _m) {
  switch(_m->GetType()) {
    case MultiBody::MultiBodyType::Active:
      {
        ActiveMultiBody* aMB = dynamic_cast<ActiveMultiBody*>(_m);
        for(size_t i = 0; i < aMB->GetNumBodies(); ++i)
          m_bodies.emplace_back(this, aMB->GetFreeBody(i));
      }
      break;
    case MultiBody::MultiBodyType::Passive:
      {
        StaticMultiBody* sMB = dynamic_cast<StaticMultiBody*>(_m);
        for(size_t i = 0; i < sMB->GetNumBodies(); ++i)
          m_bodies.emplace_back(this, sMB->GetFixedBody(i));
      }
      break;
    case MultiBody::MultiBodyType::Internal:
      break;
    default:
      throw RunTimeException(WHERE, "Unrecognized MultiBody type");
  }
}

/*---------------------------- MultiBody Support -----------------------------*/

size_t
Drawable::
GetNumBodies() const noexcept {
  return m_bodies.size();
}


void
Drawable::
PushTransform(const size_t _i, const glutils::transform& _t) {
  m_bodies[_i].push_transform(_t);
}


void
Drawable::
UpdateTransform() {
  for(auto& b : m_bodies)
    b.update_transform();
}

/*--------------------------- Drawable Overrides -----------------------------*/

void
Drawable::
draw() {
  for(auto& b : m_bodies)
    b.render();
}


void
Drawable::
draw_select() {
  for(auto& b : m_bodies)
    b.render_select();
}


void
Drawable::
draw_selected() {
  /// @TODO
}


void
Drawable::
draw_highlighted() {
  /// @TODO
}


void
Drawable::
select() noexcept {
  for(auto& body : m_bodies)
    body.glutils::drawable::select();
}


void
Drawable::
deselect() noexcept {
  for(auto& body : m_bodies)
    body.glutils::drawable::deselect();
}


void
Drawable::
highlight() noexcept {
  for(auto& body : m_bodies)
    body.glutils::drawable::highlight();
}


void
Drawable::
unhighlight() noexcept {
  for(auto& body : m_bodies)
    body.glutils::drawable::unhighlight();
}

/*----------------------------------------------------------------------------*/
